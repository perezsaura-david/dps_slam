# Copyright 2026 Universidad Politecnica de Madrid (UPM).
#
# Author: Guanliang Li
# Contributor: Pedro Espinosa Angulo, Santiago Tapia Fernandez (supervised)
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
from sklearn.cluster import DBSCAN
import cv2
import threading
import time
import struct

class LineDetector3D(Node):
    def __init__(self):
        super().__init__('line_detector_3d')

        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar_points',
            self.pointcloud_callback,
            10
        )
        
        self.pc_pub = self.create_publisher(PointCloud2, 'result_pointcloud', 10)
        
        # Config -- DBSCAN
        self.declare_parameter('dbscan_eps', 20.0) # cm
        self.declare_parameter('dbscan_min_samples', 20)
        
        # Config -- RANSAC
        self.declare_parameter('ransac_iterations', 50)
        self.declare_parameter('distance_threshold', 100) 
        
        self.eps = self.get_parameter('dbscan_eps').value
        self.min_samples = self.get_parameter('dbscan_min_samples').value
        self.iterations = self.get_parameter('ransac_iterations').value
        self.threshold = self.get_parameter('distance_threshold').value

        # Debug Image Publishing Setup
        self.declare_parameter('debug_img_flag', True)
        self.debug_img_flag = self.get_parameter('debug_img_flag').value
        if self.debug_img_flag:
            self.image_publisher = self.create_publisher(Image, 'debug_image', 10)
            self.raw_img_size = 4096
            self.img_size = 1024
            self.img_scale = self.img_size / self.raw_img_size
            
            self.debug_image = np.ones((self.img_size, self.img_size, 3), dtype=np.uint8) * 255
            self.worker_image = self.debug_image.copy()

            self.image_lock = threading.Lock()
            self.new_image_event = threading.Event()
            self.image_publish_thread = threading.Thread(target=self.image_worker_loop, daemon=True)
            self.image_publish_thread.start()
            self.get_logger().info('Image publishing thread started for debug images.')

    def world_to_pixel(self, x, y):
        u = int(self.img_size / 2 + x * self.img_scale)
        v = int(self.img_size / 2 - y * self.img_scale)
        return v, u
        
    def pack_rgb(self, r, g, b):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

    def pointcloud_callback(self, msg):
        start_time = time.time()
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_list = list(gen)
        if len(pts_list) == 0:
            self.get_logger().warning("Point cloud is empty")
            return

        pts_arr = np.array(pts_list)
        points = np.vstack([pts_arr['x'], pts_arr['y'], pts_arr['z']]).T.astype(np.float64) * 100.0

        if len(points) < self.min_samples:
            return

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)

        detected_lines = 0
        frame_endpoints = [] 
        
        output_points =[]
        noise_rgb = self.pack_rgb(100, 100, 100)

        for pt in points[labels == -1]:
            output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, noise_rgb))

        for label in unique_labels:
            if label == -1:
                continue

            cluster_points = points[labels == label]

            if len(cluster_points) < 10:
                for pt in cluster_points:
                    output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, noise_rgb))
                continue

            best_plane_point, best_plane_normal, inliers_mask = self.ransac_3d_plane(cluster_points)
            
            if best_plane_point is not None:
                inliers_count = np.sum(inliers_mask)
                inlier_ratio = inliers_count / len(cluster_points)
                
                if inlier_ratio > 0.5:
                    detected_lines += 1

                    inlier_points = cluster_points[inliers_mask]
                    outlier_points = cluster_points[~inliers_mask]
                    
                    r, g, b = np.random.randint(50, 255, 3)
                    inlier_rgb = self.pack_rgb(r, g, b)
                    outlier_rgb = self.pack_rgb(r//3, g//3, b//3)

                    for pt in inlier_points:
                        output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, inlier_rgb))
                    for pt in outlier_points:
                        output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, outlier_rgb))

                    nx, ny, nz = best_plane_normal
                    dir_2d = np.array([-ny, nx])
                    dir_norm = np.linalg.norm(dir_2d)
                    if dir_norm < 1e-6:
                        continue
                    dir_2d = dir_2d / dir_norm

                    inliers_2d = inlier_points[:, :2]
                    best_point_2d = best_plane_point[:2]

                    t = np.dot(inliers_2d - best_point_2d, dir_2d)
                    p_start = best_point_2d + np.min(t) * dir_2d
                    p_end = best_point_2d + np.max(t) * dir_2d
                    
                    frame_endpoints.append([*p_start, *p_end])
                    
                    line_len_cm = np.linalg.norm(p_end - p_start)
                    num_line_pts = int(line_len_cm) * 2
                    red_rgb = self.pack_rgb(255, 0, 0)
                    
                    if num_line_pts > 0:
                        xs = np.linspace(p_start[0], p_end[0], num_line_pts)
                        ys = np.linspace(p_start[1], p_end[1], num_line_pts)
                        z_val = best_plane_point[2] 
                        
                        for lx, ly in zip(xs, ys):
                            output_points.append((lx/100.0, ly/100.0, z_val/100.0, red_rgb))
                            output_points.append((lx/100.0, ly/100.0, (z_val+5.0)/100.0, red_rgb))
                            output_points.append((lx/100.0, ly/100.0, (z_val-5.0)/100.0, red_rgb))
                else:
                    for pt in cluster_points:
                        output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, noise_rgb))
            else:
                for pt in cluster_points:
                    output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, noise_rgb))
        
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        fields =[
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pc_msg = pc2.create_cloud(header, fields, output_points)
        self.pc_pub.publish(pc_msg)

        end_time = time.time()
        self.get_logger().info(f"Detected {detected_lines} lines | Time: {end_time - start_time:.3f} s")

        if self.debug_img_flag:
            display_img = np.ones((self.img_size, self.img_size, 3), dtype=np.uint8) * 255
            center_u, center_v = self.world_to_pixel(0, 0)
            cv2.circle(display_img, (center_u, center_v), 4, (0, 255, 0), -1)
            
            for pt in points:
                u, v = self.world_to_pixel(pt[0], pt[1])
                if 0 <= u < self.img_size and 0 <= v < self.img_size:
                    display_img[u,v] = (200, 200, 200)

            for label in unique_labels:
                if label == -1:
                    continue
                cluster_points = points[labels == label]
                display_color = (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255))
                for pt in cluster_points:
                    u, v = self.world_to_pixel(pt[0], pt[1])
                    if 0 <= u < self.img_size and 0 <= v < self.img_size:
                        display_img[u, v] = display_color

            for line in frame_endpoints:
                p_start = line[0:2]
                p_end = line[2:4]
                v1, u1 = self.world_to_pixel(p_start[0], p_start[1])
                v2, u2 = self.world_to_pixel(p_end[0], p_end[1])
                cv2.line(display_img, (u1, v1), (u2, v2), (0, 0, 255), 3)
                
            with self.image_lock:
                self.debug_image = display_img
            self.new_image_event.set()

    def ransac_3d_plane(self, points):
        num_points = len(points)
        best_inlier_count = 0
        best_inliers_mask = None
        
        for _ in range(self.iterations):
            sample_indices = np.random.choice(num_points, 3, replace=False)
            p1 = points[sample_indices[0]]
            p2 = points[sample_indices[1]]
            p3 = points[sample_indices[2]]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_val = np.linalg.norm(normal)
            
            if norm_val < 1e-6:
                continue
            normal = normal / norm_val

            vectors_to_p1 = points - p1
            distances = np.abs(np.dot(vectors_to_p1, normal))

            inliers_mask = distances < self.threshold
            inlier_count = np.sum(inliers_mask)
            
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inliers_mask = inliers_mask

        if best_inlier_count >= 3:
            inlier_points = points[best_inliers_mask]

            best_plane_point = np.mean(inlier_points, axis=0)

            centered_points = inlier_points - best_plane_point
            _, _, vh = np.linalg.svd(centered_points)
            best_plane_normal = vh[2]
            
            return best_plane_point, best_plane_normal, best_inliers_mask
            
        return None, None, None

    def image_worker_loop(self):
        while rclpy.ok():
            if self.new_image_event.wait(timeout=1.0):
                self.new_image_event.clear()
                with self.image_lock:
                    np.copyto(self.worker_image, self.debug_image)
                self.publish_ros_image(self.worker_image, encoding='bgr8')

    def publish_ros_image(self, image_np, encoding='bgr8'):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        msg.height = image_np.shape[0]
        msg.width = image_np.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image_np.tobytes()
        self.image_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()