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
from sensor_msgs.msg import Image
from msg_interfaces.msg import Objects, MapElementswithDistance

import threading
import numpy as np
from sklearn.cluster import DBSCAN
import cv2
from dataclasses import dataclass, fields

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.geometry_functions_ import *

@dataclass
class LocalFusionConfig:
    # DBSCAN configuration
    dist_threshold: float = 10.0
    angle_threshold_deg: float = 12.0
    overlap_thresh: float = 50.0 # cm
    eps: float = 0.8 # close to 1.0
    min_samples: int = 1

    # Wall connected threshold -- for corner generation
    wall_connection_threshold: float = 10.0  # cm
    # Wall length threshold -- to filter short walls
    wall_length_threshold: float = 100.0  # cm
    
    # Image configuration
    publish_img_flag: bool = True
    processed_img_size: int = 640
    base_img_size: int = 4096
    sensor_posx: int = 2048
    sensor_posy: int = 2048


class LocalFusionNode(Node):
    def __init__(self):
        super().__init__('local_fusion_node')

        self.config = LocalFusionConfig()
        self.get_logger().info("Loading Parameters...")
        param_log_str = "LOCAL FUSION CONFIGURATION:\n"

        for field in fields(LocalFusionConfig):
            self.declare_parameter(field.name, field.default)
            value = self.get_parameter(field.name).value
            setattr(self.config, field.name, value)
            param_log_str += f"\t{field.name}: {value}\n"

        self.get_logger().info(param_log_str)
        self.angle_threshold = np.deg2rad(self.config.angle_threshold_deg)

        self.subscription = self.create_subscription(
            Objects,
            'detection_results',
            self.listener_callback,
            10)
        
        self.local_fused_map_publisher = self.create_publisher(MapElementswithDistance, 'local_fused_map', 10)
        self.image_publisher = self.create_publisher(Image, 'fused_map_image', 10)

        # Image initialization
        self.debug_image = Image()
        self.results_image = np.ones((self.config.processed_img_size, self.config.processed_img_size, 3), dtype=np.uint8) * 255

        # Image publishment thread
        self.image_lock = threading.Lock()
        self.new_image_event = threading.Event()
        self.image_publish_thread = threading.Thread(target=self.image_worker_loop, daemon=True)
        self.image_publish_thread.start()

    def image_worker_loop(self):
        while rclpy.ok():
            if self.new_image_event.wait(timeout=1.0):
                self.new_image_event.clear()
                with self.image_lock:
                    fused_image = self.debug_image
                if fused_image.data:
                    self.image_publisher.publish(fused_image)

    def listener_callback(self, msg):
        self.get_logger().info('Local Fusion callback triggered.')
        # Fused map elements message
        fused_map = MapElementswithDistance()
        fused_map.header = msg.header
        # Convert the walls to polar coordinate elements
        endpoints = np.array(msg.endpoints, dtype=np.float32).reshape(-1, 4) # x1, y1, x2, y2
        if len(endpoints) == 0:
            self.get_logger().info('No endpoints received for fusion.')
            return
        
        # Skeleton wall for method 'LSD' and 'Hough'
        if msg.method in ['lsd_detector', 'hought_detector']:
            endpoints = merge_wall_envelopes(endpoints)
        
        polar_list = []
        valid_indices = []
        for idx, wall in enumerate(endpoints):
            try:
                polar_coords: np.ndarray = endpoints2polar(wall)
                if abs(polar_coords[3] - polar_coords[2]) > self.config.wall_length_threshold:
                    polar_list.append(polar_coords)
                    valid_indices.append(idx)
            except ValueError as e:
                self.get_logger().warn(f'Invalid wall endpoints at index {idx}: {e}')
        valid_endpoints = endpoints[valid_indices]
        # DBSCAN Clustering
        polar_data = np.array(polar_list, dtype=np.float32)  # Shape: (N, 4)
        dist_matrix = self.compute_distance_matrix(valid_endpoints)
        clustering = DBSCAN(eps=self.config.eps, min_samples=self.config.min_samples, metric='precomputed')
        labels = clustering.fit_predict(dist_matrix)
        # Fuse based on clusters
        fused_endpoints = []
        fused_polar_params = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            class_member_mask = (labels == label)
            cluster_data = polar_data[class_member_mask]
            # Simple fusion: average the polar coordinates
            mean_d = np.mean(cluster_data[:, 0])
            sin_sum = np.sum(np.sin(cluster_data[:, 1]))
            cos_sum = np.sum(np.cos(cluster_data[:, 1]))
            mean_alpha = np.arctan2(sin_sum, cos_sum) % (2 * np.pi)

            p0_x = mean_d * np.cos(mean_alpha)
            p0_y = mean_d * np.sin(mean_alpha)
            dir_x = -np.sin(mean_alpha)
            dir_y = np.cos(mean_alpha)
            projected_lengths = []
            for row in cluster_data:
                raw_endpoints = polar2endpoints(row)
                x1, y1, x2, y2 = raw_endpoints
                proj_1 = (x1 - p0_x) * dir_x + (y1 - p0_y) * dir_y
                proj_2 = (x2 - p0_x) * dir_x + (y2 - p0_y) * dir_y
                projected_lengths.append(proj_1)
                projected_lengths.append(proj_2)
            min_d1 = min(projected_lengths)
            max_d2 = max(projected_lengths)
            fused_endpoints.append([mean_d, mean_alpha, min_d1, max_d2])
            # wall_len = max_d2 - min_d1

            wall_segment: np.ndarray = polar2endpoints(np.array([mean_d, mean_alpha, min_d1, max_d2]))
            fused_polar_params.append([mean_d, mean_alpha, *wall_segment])

        fused_map.num_walls = len(fused_endpoints)
        fused_map.wall_parameters = np.array(fused_endpoints, dtype=np.float32).flatten().tolist()

        # Corner generation
        corners = []
        num_walls = len(fused_endpoints)
        fused_params_array = np.array(fused_polar_params, dtype=np.float32)
        for i in range(num_walls):
            for j in range(i + 1, num_walls):
                w1 = fused_params_array[i] # d, alpha, x1, y1, x2, y2
                w2 = fused_params_array[j]

                intersection = calculate_line_intersection(w1[0], w1[1], w2[0], w2[1])
                if intersection is None:
                    continue # Parallel lines
                x_int, y_int = intersection
                # dist_to_w1 = point_to_segment_distance(x_int, y_int, w1[2], w1[3], w1[4], w1[5])
                # dist_to_w2 = point_to_segment_distance(x_int, y_int, w2[2], w2[3], w2[4], w2[5])
                dist_to_w1 = distance_point_to_line_segment(np.array([x_int, y_int]), 
                                                            np.array([w1[2], w1[3]]), 
                                                            np.array([w1[4], w1[5]]))
                dist_to_w2 = distance_point_to_line_segment(np.array([x_int, y_int]), 
                                                            np.array([w2[2], w2[3]]),
                                                            np.array([w2[4], w2[5]]))
                if dist_to_w1 <= self.config.wall_connection_threshold and dist_to_w2 <= self.config.wall_connection_threshold:
                    corners.append([x_int, y_int])
        fused_map.num_corners = len(corners)
        fused_map.corners_points = np.array(corners, dtype=np.float32).flatten().tolist()
        
        # Column data
        if len(msg.columns_circles_centers) > 0:
            fused_map.num_columns = len(msg.columns_circles_radius)
            # fused_map.columns_xyr
            centers = np.array(msg.columns_circles_centers, dtype=np.float32).reshape(-1, 2)
            radii = np.array(msg.columns_circles_radius, dtype=np.float32).reshape(-1, 1)
            combined = np.hstack((centers, radii)).flatten().tolist()
            fused_map.columns_xyr = combined

        # Others data
        # if len(msg.others_circles_centers) > 0:
        #     fused_map.num_columns += len(msg.others_circles_radius)
        #     centers = np.array(msg.others_circles_centers, dtype=np.float32).reshape(-1, 2)
        #     radii = np.array(msg.others_circles_radius, dtype=np.float32).reshape(-1, 1)
        #     combined = np.hstack((centers, radii)).flatten().tolist()
        #     fused_map.columns_xyr.extend(combined)

        # Populate fused_map based on received msg
        self.local_fused_map_publisher.publish(fused_map)
        if self.config.publish_img_flag:
            self.debug_image_drawing(endpoints, fused_map)
            self.new_image_event.set()

    def compute_distance_matrix(self, endpoints: np.ndarray) -> np.ndarray:
        n = len(endpoints)
        dist_matrix = np.zeros((n, n), dtype=np.float32)

        incli_vals = []
        for wall in endpoints:
            x1, y1, x2, y2 = wall
            if abs(x2 - x1) > 1e-6:
                incli = np.arctan2((y2 - y1), (x2 - x1)) % (2 * np.pi)
            else:
                incli = np.pi / 2
            incli_vals.append(incli)

        dist_thresh = max(self.config.dist_threshold, 1e-6)
        angle_thresh = max(self.angle_threshold, 1e-6)

        for i in range(n):
            for j in range(i + 1, n):
                w1 = endpoints[i]
                w2 = endpoints[j]
                x1_1, y1_1, x2_1, y2_1 = w1
                x1_2, y1_2, x2_2, y2_2 = w2
                P1 = np.array([x1_1, y1_1])
                P2 = np.array([x2_1, y2_1])
                Q1 = np.array([x1_2, y1_2])
                Q2 = np.array([x2_2, y2_2])
                # Calculate distance between point and line
                dis1_1 = distance_point_to_line(P1, Q1, Q2)
                dis1_2 = distance_point_to_line(P2, Q1, Q2)
                dis2_1 = distance_point_to_line(Q1, P1, P2)
                dis2_2 = distance_point_to_line(Q2, P1, P2)
                delta_dis = min(dis1_1, dis1_2, dis2_1, dis2_2)
                # Compute angular difference considering wrap-around
                diff_angle = abs(incli_vals[i] - incli_vals[j])
                delta_angle = min(diff_angle, np.pi - diff_angle)
                # Calculate overlap distance
                overlap_distance = overlap_cal(w1, w2)
                
                term_dis = delta_dis / dist_thresh
                term_angle = angle_logistic_function(delta_angle, angle_thresh)
                term_ove = overlap_distance / self.config.overlap_thresh
                dist = max(term_dis, term_angle, term_ove)

                dist_matrix[i, j] = dist
                dist_matrix[j, i] = dist

        return dist_matrix
    
    def debug_image_drawing(self, endpoints_list: np.ndarray, fused_map: MapElementswithDistance):
        scale_ratio: float = self.config.processed_img_size / self.config.base_img_size 
        canvas = self.results_image.copy()
        # Detector original image
        for endpoints in endpoints_list:
            x1, y1, x2, y2 = endpoints
            p1_img = (self.laser_to_img(np.array([x1, y1])) * scale_ratio).flatten()
            p2_img = (self.laser_to_img(np.array([x2, y2])) * scale_ratio).flatten()
            cv2.line(canvas, (int(p1_img[0]), int(p1_img[1])), 
                     (int(p2_img[0]), int(p2_img[1])), (255, 0, 0), 2)
            cv2.putText(canvas, 'W', (int((p1_img[0]+p2_img[0])/2), int((p1_img[1]+p2_img[1])/2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
        # Fused walls
        for i in range(0, len(fused_map.wall_parameters), 4):
            d, alpha, d1, d2 = fused_map.wall_parameters[i:i+4]
            wall_endpoints = polar2endpoints(np.array([d, alpha, d1, d2]))
            x1, y1, x2, y2 = wall_endpoints
            p1_img = (self.laser_to_img(np.array([x1, y1])) * scale_ratio).flatten()
            p2_img = (self.laser_to_img(np.array([x2, y2])) * scale_ratio).flatten()
            cv2.line(canvas, (int(p1_img[0]), int(p1_img[1])), 
                     (int(p2_img[0]), int(p2_img[1])), (0, 0, 255), 1)
            cv2.putText(canvas, 'F', (int((p1_img[0]+p2_img[0])/2), int((p1_img[1]+p2_img[1])/2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
        # Corners
        for i in range(0, len(fused_map.corners_points), 2):
            center_x = fused_map.corners_points[i]
            center_y = fused_map.corners_points[i+1]
            center_img = (self.laser_to_img(np.array([center_x, center_y])) * scale_ratio).flatten()
            cv2.circle(canvas, (int(center_img[0]), int(center_img[1])), 
                       5, (0, 255, 255), -1)
            
        # Columns and others
        for i in range(0, len(fused_map.columns_xyr), 3):
            center_x = fused_map.columns_xyr[i]
            center_y = fused_map.columns_xyr[i+1]
            radius = fused_map.columns_xyr[i+2]
            center_img = (self.laser_to_img(np.array([center_x, center_y])) * scale_ratio).flatten()
            cv2.circle(canvas, (int(center_img[0]), int(center_img[1])), 
                       int(radius * scale_ratio), (0, 255, 0), 2)
            
        # Convert to ROS Image message
        fused_img = Image()
        fused_img.header.stamp = self.get_clock().now().to_msg()
        fused_img.header.frame_id = 'lidar_frame'
        fused_img.height = self.config.processed_img_size
        fused_img.width = self.config.processed_img_size
        fused_img.encoding = 'bgr8'
        fused_img.is_bigendian = 0
        fused_img.step = self.config.processed_img_size * 3
        fused_img.data = canvas.tobytes()
        
        with self.image_lock:
            self.debug_image = fused_img

    def laser_to_img(self, vertices_laser: np.ndarray) -> np.ndarray:
        x_img = vertices_laser[0] + self.config.sensor_posx
        y_img = self.config.sensor_posy - vertices_laser[1]

        return np.column_stack((x_img, y_img))
    
def main(args=None):
    rclpy.init(args=args)
    local_fusion_node = LocalFusionNode()
    rclpy.spin(local_fusion_node)
    local_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
