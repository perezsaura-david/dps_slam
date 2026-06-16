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
import cv2
import threading
import time
import struct
from scipy.spatial import cKDTree
from collections import deque

class RegionGrowing3DNode(Node):
    def __init__(self):
        super().__init__('region_growing_3d')

        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar_points',
            self.pointcloud_callback,
            10
        )
        
        self.pc_pub = self.create_publisher(PointCloud2, 'result_pointcloud', 10)
        
        # Region Growing Config
        self.declare_parameter('k_search', 100)
        self.declare_parameter('angle_threshold', 2.0)
        self.declare_parameter('curvature_threshold', 0.02)
        self.declare_parameter('min_cluster_size', 30)
        
        self.k_search = self.get_parameter('k_search').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.curv_threshold = self.get_parameter('curvature_threshold').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value

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
            return

        pts_arr = np.array(pts_list)
        points = np.vstack([pts_arr['x'], pts_arr['y'], pts_arr['z']]).T.astype(np.float64) * 100.0

        num_points = len(points)
        if num_points < self.k_search:
            return

        tree = cKDTree(points)
        _, nn_indices = tree.query(points, k=self.k_search)
        
        neighbors = points[nn_indices]
        centroids = np.mean(neighbors, axis=1, keepdims=True)
        centered = neighbors - centroids
        
        covariances = np.matmul(centered.transpose(0, 2, 1), centered)
        
        eigenvalues, eigenvectors = np.linalg.eigh(covariances)
        
        normals = eigenvectors[:, :, 0]
        curvatures = eigenvalues[:, 0] / (np.sum(eigenvalues, axis=1) + 1e-6)

        labels = -np.ones(num_points, dtype=int)
        visited = np.zeros(num_points, dtype=bool)
        
        sorted_indices = np.argsort(curvatures)
        
        cos_thresh = np.cos(np.radians(self.angle_threshold))
        cluster_id = 0

        for seed_idx in sorted_indices:
            if visited[seed_idx]:
                continue
            
            if curvatures[seed_idx] > self.curv_threshold:
                continue

            seed_queue = deque([seed_idx])
            visited[seed_idx] = True
            current_region =[]

            while seed_queue:
                curr_idx = seed_queue.popleft()
                current_region.append(curr_idx)
                labels[curr_idx] = cluster_id

                for nbr_idx in nn_indices[curr_idx]:
                    if visited[nbr_idx]:
                        continue

                    dot_product = abs(np.dot(normals[seed_idx], normals[nbr_idx]))
                    
                    if dot_product > cos_thresh:
                        labels[nbr_idx] = cluster_id
                        visited[nbr_idx] = True
                        
                        if curvatures[nbr_idx] < self.curv_threshold:
                            seed_queue.append(nbr_idx)

            if len(current_region) >= self.min_cluster_size:
                cluster_id += 1
            else:
                for idx in current_region:
                    labels[idx] = -1

        unique_labels = set(np.unique(labels))
        self.get_logger().info(f"Region Growing found {len(unique_labels) - (1 if -1 in unique_labels else 0)} clusters.")

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

            centroid = np.mean(cluster_points, axis=0)
            centered_points = cluster_points - centroid
            _, _, vh = np.linalg.svd(centered_points)
            best_plane_normal = vh[2]

            if abs(best_plane_normal[2]) > 0.5:
                continue

            detected_lines += 1

            r, g, b = np.random.randint(50, 255, 3)
            cluster_rgb = self.pack_rgb(r, g, b)

            for pt in cluster_points:
                output_points.append((pt[0]/100.0, pt[1]/100.0, pt[2]/100.0, cluster_rgb))

            nx, ny, nz = best_plane_normal
            dir_2d = np.array([-ny, nx])
            dir_norm = np.linalg.norm(dir_2d)
            if dir_norm < 1e-6:
                continue
            dir_2d = dir_2d / dir_norm

            inliers_2d = cluster_points[:, :2]
            best_point_2d = centroid[:2]

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
                z_val = centroid[2] 
                
                for lx, ly in zip(xs, ys):
                    output_points.append((lx/100.0, ly/100.0, z_val/100.0, red_rgb))
                    output_points.append((lx/100.0, ly/100.0, (z_val+5.0)/100.0, red_rgb))
                    output_points.append((lx/100.0, ly/100.0, (z_val-5.0)/100.0, red_rgb))
        
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
        self.get_logger().info(f"Processed {num_points} points | Detected {detected_lines} walls | Time: {end_time - start_time:.3f} s")

        if self.debug_img_flag:
            display_img = np.ones((self.img_size, self.img_size, 3), dtype=np.uint8) * 255
            center_u, center_v = self.world_to_pixel(0, 0)
            cv2.circle(display_img, (center_u, center_v), 4, (0, 255, 0), -1)
            
            for pt in points:
                u, v = self.world_to_pixel(pt[0], pt[1])
                if 0 <= u < self.img_size and 0 <= v < self.img_size:
                    display_img[u,v] = (200, 200, 200)

            for label in unique_labels:
                if label == -1: continue
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
    node = RegionGrowing3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()