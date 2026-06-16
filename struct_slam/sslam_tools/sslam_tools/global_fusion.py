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
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose
from msg_interfaces.msg import MapElementswithDistance
from msg_interfaces.msg import Wall
import tf2_ros

from geometry_msgs.msg import PoseWithCovariance
from dps_slam_msgs.msg import DetectionWithIDArray, DetectionWithID, Geometry
from dps_slam_msgs.msg import Line2D, Cylinder

import numpy as np
import threading
import cv2
import math
from dataclasses import dataclass, fields
from scipy.optimize import linear_sum_assignment

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.global_fusion_class import GlobalWall, GlobalColumn, ManhattanWorldOptimizer
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.geometry_functions_ import polar2endpoints, distance_point_to_line

@dataclass
class GlobalFusionConfig:
    # Threshold config
    match_dist_thresh: float = 30.0 # cm
    match_angle_thresh_deg: float = 10.0 # deg
    overlap_thresh: float = 20.0 # cm
    gate_threshold: float = 100.0
    max_misses_thresh: int = 3
    sensor_range: float = 3000.0 # cm
    dist_weight: float = 1 / sensor_range
    angle_weight: float = 1 / (2 * np.pi)

    # Image configuration
    publish_img_flag: bool = True
    processed_img_size: int = 640
    base_img_size: int = 4096
    sensor_posx: int = 2048
    sensor_posy: int = 2048

    # Movement config
    movement_flag: bool = True
    map_frame: str = 'drone0/map'
    lidar_frame: str = 'drone0/base_link'

    # Manhattan world optimization
    manhattan_opt_flag: bool = False

    corner_max_gap: float = 80.0 # cm, maximum gap for a corner to be considered valid during SVD correction
    svd_lambda_reg: float = 0.5 # Regularization term for SVD correction, higher values lead to more conservative corrections
    stable_hits_threshold: int = 2 # Minimum hits for a global wall to be considered stable and used for yaw correction and SVD weighting

class GlobalFusionNode(Node):
    def __init__(self):
        super().__init__('global_fusion_node')
        self.config = GlobalFusionConfig()

        self.get_logger().info("Loading Parameters...")
        param_log_str = "GLOBAL FUSION CONFIGURATION:\n"

        for field in fields(GlobalFusionConfig):
            self.declare_parameter(field.name, field.default)
            value = self.get_parameter(field.name).value
            setattr(self.config, field.name, value)
            param_log_str += f"\t{field.name}: {value}\n"

        self.get_logger().info(param_log_str)
        self.match_angle_thresh = np.deg2rad(self.config.match_angle_thresh_deg)

        # TF initialization
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_msg_time = None

        # Record last TF for movement compensation
        self.last_tf_x = None
        self.last_tf_y = None
        self.last_tf_yaw = None
        self.is_first_frame = True

        self.subscription = self.create_subscription(
            MapElementswithDistance,
            'local_fused_map',
            self.listener_callback,
            10)

        # Global map storage
        self.next_wall_id: int = 0
        self.global_walls: list[GlobalWall] = [] # List of GlobalWall instances
        self.next_column_id: int = 0
        self.global_columns: list[GlobalColumn] = [] # List of GlobalColumn instances
        self.active_debug_corners = []
        self.robot_trajectory = []

        # Manhattan world optimizer
        if self.config.manhattan_opt_flag:
            self.optimizer = ManhattanWorldOptimizer()

        # Image initialization
        self.image_publisher = self.create_publisher(Image, 'global_fused_map_image', 10)
        self.debug_image = Image()
        self.image_lock = threading.Lock()
        self.new_image_event = threading.Event()
        self.image_publish_thread = threading.Thread(target=self.image_worker_loop, daemon=True)
        self.image_publish_thread.start()
        self.render_data = None

        # G2O Publisher
        self.graph_opt_publisher =self.create_publisher(DetectionWithIDArray, 'semantic_observations', 10)

        self.get_logger().info('Global Fusion Node initialized.')

    def image_worker_loop(self):
        while rclpy.ok():
            if self.new_image_event.wait(timeout=1.0):
                self.new_image_event.clear()
                with self.image_lock:
                    if self.render_data is None: continue
                    inc_walls, g_walls, g_cols, r_x, r_y, active_corners, trajectory = self.render_data

                fused_image_msg = self._do_debug_image_drawing(inc_walls, g_walls, g_cols, r_x, r_y, active_corners, trajectory)
                self.image_publisher.publish(fused_image_msg)

    def transform_incoming_data(self, incoming_walls: np.ndarray, incoming_columns: np.ndarray):
        transformed_walls = []
        transformed_columns = []

        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)

        # Transform walls
        for row in incoming_walls:
            endpoints = polar2endpoints(row)
            new_x1 = cos_y * endpoints[0] - sin_y * endpoints[1] + self.robot_x
            new_y1 = sin_y * endpoints[0] + cos_y * endpoints[1] + self.robot_y
            new_x2 = cos_y * endpoints[2] - sin_y * endpoints[3] + self.robot_x
            new_y2 = sin_y * endpoints[2] + cos_y * endpoints[3] + self.robot_y

            # Convert back to polar form
            dir_x = new_x2 - new_x1
            dir_y = new_y2 - new_y1
            new_theta = math.atan2(-dir_x, dir_y) % (2 * math.pi)
            new_rho = new_x1 * math.cos(new_theta) + new_y1 * math.sin(new_theta)
            if new_rho < 0:
                new_rho = -new_rho
                new_theta = (new_theta + math.pi) % (2 * math.pi)
            center_x, center_y = new_rho * np.cos(new_theta), new_rho * np.sin(new_theta)
            dir_vec = np.array([-np.sin(new_theta), np.cos(new_theta)])
            new_d1 = np.dot([new_x1 - center_x, new_y1 - center_y], dir_vec)
            new_d2 = np.dot([new_x2 - center_x, new_y2 - center_y], dir_vec)
            if new_d1 > new_d2:
                new_d1, new_d2 = new_d2, new_d1
            transformed_walls.append([new_rho, new_theta, new_d1, new_d2])

        # Transform columns
        for row in incoming_columns:
            new_x = cos_y * row[0] - sin_y * row[1] + self.robot_x
            new_y = sin_y * row[0] + cos_y * row[1] + self.robot_y
            transformed_columns.append([new_x, new_y, row[2]])

        return  np.array(transformed_walls, dtype=np.float32).reshape(-1, 4), \
                np.array(transformed_columns, dtype=np.float32).reshape(-1, 3)

    def diff_calc(self, new_wall: np.ndarray, g_wall: np.array) -> float:
        endpoints_new = polar2endpoints(new_wall)
        endpoints_g = polar2endpoints(g_wall)
        x1_1, y1_1, x2_1, y2_1 = endpoints_new
        x1_2, y1_2, x2_2, y2_2 = endpoints_g
        P1 = np.array([x1_1, y1_1])
        P2 = np.array([x2_1, y2_1])
        Q1 = np.array([x1_2, y1_2])
        Q2 = np.array([x2_2, y2_2])
        dis1_1 = distance_point_to_line(P1, Q1, Q2)
        dis1_2 = distance_point_to_line(P2, Q1, Q2)
        dis2_1 = distance_point_to_line(Q1, P1, P2)
        dis2_2 = distance_point_to_line(Q2, P1, P2)
        delta_dis = min(dis1_1, dis1_2, dis2_1, dis2_2)
        incli_new = np.arctan2(y2_1 - y1_1, x2_1 - x1_1) % (2 * np.pi)
        incli_g = np.arctan2(y2_2 - y1_2, x2_2 - x1_2) % (2 * np.pi)
        diff = abs(incli_new - incli_g) % np.pi
        if diff > np.pi / 2:
            diff = np.pi - diff
        delta_angle = diff
        # Overlap calculation
        g_theta = g_wall[1]
        line_dir = np.array([-np.sin(g_theta), np.cos(g_theta)]) # Line direction vector
        proj_P1 = np.dot(P1, line_dir)
        proj_P2 = np.dot(P2, line_dir)
        proj_Q1 = np.dot(Q1, line_dir)
        proj_Q2 = np.dot(Q2, line_dir)
        range_new = sorted([proj_P1, proj_P2])
        range_g = sorted([proj_Q1, proj_Q2])
        overlap = min(range_new[1], range_g[1]) - max(range_new[0], range_g[0])
        if overlap > 0:
            delta_distance = 0.0
        else:
            delta_distance = abs(overlap)
        return delta_dis, delta_angle, delta_distance
    
    def compute_cost(self, local_w: np.ndarray, global_w: np.ndarray) -> float:
        d_diff, a_diff, delta_distance = self.diff_calc(local_w, global_w)

        if d_diff > self.config.match_dist_thresh or a_diff > self.match_angle_thresh:
            return np.inf
        if delta_distance > self.config.overlap_thresh:
            return np.inf
        
        dist_cost = d_diff / self.config.match_dist_thresh
        angle_cost = a_diff / self.match_angle_thresh
        
        cost = dist_cost + angle_cost
        return cost
    
    def manhattan_pre_filter(self, local_walls_data: np.ndarray) -> np.ndarray:
        if not self.config.manhattan_opt_flag or len(self.global_walls) < 3:
            return local_walls_data
        
        dom_theta = self.optimizer.find_dominant_direction(self.global_walls)
        filtered_walls = []
        for l_w in local_walls_data:
            global_theta = (l_w[1] + self.robot_yaw) % (2 * np.pi)
            diff = (global_theta - dom_theta) % (2 * np.pi)
            k = round(diff / (np.pi / 2))
            target_theta = (dom_theta + k * (np.pi / 2)) % (2 * np.pi)
            actual_diff = abs(np.arctan2(np.sin(global_theta - target_theta), np.cos(global_theta - target_theta)))
            
            if actual_diff < np.deg2rad(5.0):
                filtered_walls.append(l_w)

        return np.array(filtered_walls, dtype=np.float32).reshape(-1, 4)
    
    def get_corner_intersection(self, w1:np.ndarray, w2:np.ndarray, max_gap: float = 80.0):
        # w: [rho, theta, d1, d2]
        denom = np.sin(w2[1] - w1[1])
        if abs(denom) < 1e-5:
            return None
        
        ix = (w1[0] * np.sin(w2[1]) - w2[0] * np.sin(w1[1])) / denom
        iy = (w2[0] * np.cos(w1[1]) - w1[0] * np.cos(w2[1])) / denom

        def dist_to_segment(ix, iy, w):
            proj = -ix * np.sin(w[1]) + iy * np.cos(w[1])
            d_min, d_max = min(w[2], w[3]), max(w[2], w[3])
            if proj < d_min: return d_min - proj
            if proj > d_max: return proj -d_max
            return 0.0
        
        if dist_to_segment(ix, iy, w1) < max_gap and dist_to_segment(ix, iy, w2) < max_gap:
            return np.array([ix, iy])
        return None

    def global_data_association_and_correction(self, raw_local_walls:np.ndarray) -> tuple:
        self.active_debug_corners.clear()
        
        candidate_globals = self.global_walls

        if len(candidate_globals) == 0 or len(raw_local_walls) == 0:
            return [], list(range(len(raw_local_walls)))
        
        pred_global_walls, _ = self.transform_incoming_data(raw_local_walls, np.array([]).reshape(0,3))

        N = len(pred_global_walls)
        M = len(candidate_globals)
        gate = self.config.gate_threshold

        cost_matrix = np.full((N + M, N + M), np.inf)

        for i in range(N):
            for j in range(M):
                gw_array = np.array([candidate_globals[j].rho, candidate_globals[j].theta, candidate_globals[j].d1, candidate_globals[j].d2])
                cost = self.compute_cost(pred_global_walls[i], gw_array)
                if cost < gate:
                    cost_matrix[i, j] = cost

        for i in range(N): cost_matrix[i, M + i] = gate
        for j in range(M): cost_matrix[N + j, j] = gate
        cost_matrix[N:, M:] = 0.0

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matched_pairs = []
        unmatched_local_indices = []

        for r, c in zip(row_ind, col_ind):
            if r < N and c < M:
                if cost_matrix[r, c] < np.inf:
                    matched_pairs.append((r, c))
                else:
                    unmatched_local_indices.append(r)
            elif r < N and c >= M:
                unmatched_local_indices.append(r)

        yaw_errors = []
        for l_idx, g_idx in matched_pairs:
            if candidate_globals[g_idx].hits >= 2:
                l_theta = raw_local_walls[l_idx][1]
                g_theta = candidate_globals[g_idx].theta
                e_yaw = g_theta - l_theta - self.robot_yaw
                e_yaw = (e_yaw + np.pi) % (2 * np.pi) - np.pi
                if e_yaw > np.pi / 2: e_yaw -= np.pi
                elif e_yaw < -np.pi / 2: e_yaw += np.pi
                yaw_errors.append(e_yaw)
        
        if yaw_errors:
            delta_yaw = np.median(yaw_errors)
            if abs(delta_yaw) < np.deg2rad(15.0):
                self.robot_yaw = (self.robot_yaw + delta_yaw * 0.1) % (2 * np.pi)

        Aw, Bw = [], []
        for l_idx, g_idx in matched_pairs:
            g_wall = candidate_globals[g_idx]
            if g_wall.hits >= 2:
                l_tho = raw_local_walls[l_idx][0]
                l_theta = raw_local_walls[l_idx][1]

                temp_g_theta = l_theta + self.robot_yaw
                nx, ny = np.cos(temp_g_theta), np.sin(temp_g_theta)
                expected_rho = l_tho + self.robot_x * nx + self.robot_y * ny
                error_rho = g_wall.rho - expected_rho

                weight = (g_wall.d2 - g_wall.d1) / 100.0
                Aw.append([nx * weight, ny * weight])
                Bw.append(error_rho * weight)

        num_matches = len(matched_pairs)
        corner_weight = 5.0
        for i in range(num_matches):
            for j in range(i + 1, num_matches):
                l_idx1, g_idx1 = matched_pairs[i]
                l_idx2, g_idx2 = matched_pairs[j]
                g_wall1 = candidate_globals[g_idx1]
                g_wall2 = candidate_globals[g_idx2]
                if g_wall1.hits < 2 or g_wall2.hits < 2: continue

                l_w1, l_w2 = raw_local_walls[l_idx1], raw_local_walls[l_idx2]

                angle_diff = abs(l_w1[1] - l_w2[1]) % np.pi
                if angle_diff > np.pi / 2: angle_diff = np.pi - angle_diff
                if angle_diff < np.deg2rad(30.0): continue # 30 ~ 150 degree

                local_pt = self.get_corner_intersection(l_w1, l_w2, max_gap=self.config.corner_max_gap)
                if local_pt is None: continue

                g_w1_arr = np.array([g_wall1.rho, g_wall1.theta, g_wall1.d1, g_wall1.d2])
                g_w2_arr = np.array([g_wall2.rho, g_wall2.theta, g_wall2.d1, g_wall2.d2])
                global_pt = self.get_corner_intersection(g_w1_arr, g_w2_arr, max_gap=self.config.corner_max_gap)
                if global_pt is None: continue

                lx, ly = local_pt
                cos_y, sin_y = np.cos(self.robot_yaw), np.sin(self.robot_yaw)
                exp_x = lx * cos_y - ly * sin_y + self.robot_x
                exp_y = lx * sin_y + ly * cos_y + self.robot_y

                gx, gy = global_pt

                Aw.append([1.0 * corner_weight, 0.0])
                Bw.append((gx - exp_x) * corner_weight)
                Aw.append([0.0, 1.0 * corner_weight])
                Bw.append((gy - exp_y) * corner_weight)
                self.active_debug_corners.append(global_pt)

        Aw, Bw = np.array(Aw), np.array(Bw)

        if len(Aw) >= 2:
            U, S, Vt = np.linalg.svd(Aw, full_matrices=False)
            inv_S = np.zeros_like(S)

            for i in range(len(S)):
                if S[i] > 0.1:
                    inv_S[i] = S[i] / (S[i]**2 + self.config.svd_lambda_reg)
            
            delta_pos = Vt.T @ np.diag(inv_S) @ U.T @ Bw
            correction_dist = np.hypot(delta_pos[0], delta_pos[1])
            if correction_dist < 20.0:
                self.robot_x += delta_pos[0]
                self.robot_y += delta_pos[1]

        final_matches = [(l_idx, candidate_globals[g_idx].id) for l_idx, g_idx in matched_pairs]
        return final_matches, unmatched_local_indices
    
    def listener_callback(self, msg):
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        # Movement compensation using TF
        if self.config.movement_flag and self.last_msg_time is not None:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame=self.config.map_frame,
                    source_frame=self.config.lidar_frame,
                    time=current_time,
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )

                raw_x = trans.transform.translation.x * 100.0 # m -> cm
                raw_y = trans.transform.translation.y * 100.0
                q = trans.transform.rotation
                raw_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                if self.is_first_frame:
                    self.robot_x, self.robot_y, self.robot_yaw = raw_x, raw_y, raw_yaw
                    self.last_tf_x, self.last_tf_y, self.last_tf_yaw = raw_x, raw_y, raw_yaw
                    self.is_first_frame = False
                else:
                    dx = raw_x - self.last_tf_x
                    dy = raw_y - self.last_tf_y
                    dyaw = raw_yaw - self.last_tf_yaw
                    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

                    self.last_tf_x, self.last_tf_y, self.last_tf_yaw = raw_x, raw_y, raw_yaw

                    self.robot_x += dx
                    self.robot_y += dy
                    self.robot_yaw += dyaw
                    self.robot_yaw %= (2 * math.pi)

            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f"TF transform failed: {ex}")
                pass

        self.robot_trajectory.append((self.robot_x, self.robot_y))

        self.last_msg_time = current_time

        local_walls_data = np.array(msg.wall_parameters, dtype=np.float32).reshape(-1, 4)
        local_columns_data = np.array(msg.columns_xyr, dtype=np.float32).reshape(-1, 3)
        
        # Manhattan pre-filtering
        local_walls_data = self.manhattan_pre_filter(local_walls_data)

        # Data Association and SVD Correction
        matched_pairs_info, unmatched_local_indices = self.global_data_association_and_correction(local_walls_data)

        # Map update
        final_global_walls_data, final_global_cols_data = self.transform_incoming_data(local_walls_data, local_columns_data)
        
        # Process walls' elements
        matched_global_ids = set()
        current_frame_wall_obs = []
        # Update global walls with matched pairs info
        for l_idx, global_id in matched_pairs_info:
            target_wall = next((w for w in self.global_walls if w.id == global_id), None)
            if target_wall:
                new_rho, new_theta, new_d1, new_d2 = final_global_walls_data[l_idx]
                target_wall.update(new_rho, new_theta)
                target_wall.recalculate_endpoints_on_line(polar2endpoints(final_global_walls_data[l_idx]))
                matched_global_ids.add(target_wall.id)

                current_frame_wall_obs.append((target_wall.id, local_walls_data[l_idx], target_wall.covariance_mea))

        for l_idx in unmatched_local_indices:
            new_rho, new_theta, new_d1, new_d2 = final_global_walls_data[l_idx]
            new_wall = GlobalWall(self.next_wall_id, new_rho, new_theta, new_d1, new_d2)
            self.global_walls.append(new_wall)
            matched_global_ids.add(new_wall.id)

            current_frame_wall_obs.append((new_wall.id, local_walls_data[l_idx], new_wall.covariance_mea))
            self.next_wall_id += 1

        # Process columns' elements
        matched_column_ids = set()
        current_frame_column_obs = []
        for i, row in enumerate(final_global_cols_data):
            new_x, new_y, new_radius = row
            best_match_idx = -1
            best_match_dist = float('inf')
            for j, g_column in enumerate(self.global_columns):
                dist = np.hypot(new_x - g_column.x, new_y - g_column.y)
                if dist < self.config.match_dist_thresh:
                    if dist < best_match_dist:
                        best_match_dist = dist
                        best_match_idx = j
            if best_match_idx != -1:
                target_column = self.global_columns[best_match_idx]
                target_column.update(new_x, new_y, new_radius)
                matched_column_ids.add(target_column.id)
                current_frame_column_obs.append((target_column.id, local_columns_data[i]))
            else:
                new_column = GlobalColumn(self.next_column_id, new_x, new_y, new_radius)
                self.global_columns.append(new_column)
                current_frame_column_obs.append((new_column.id, local_columns_data[i]))
                self.next_column_id += 1
                matched_column_ids.add(new_column.id)

        # Lifecycle management
        walls_to_keep = []
        for g_wall in self.global_walls:
            if g_wall.id in matched_global_ids:
                g_wall.misses = 0
                walls_to_keep.append(g_wall)
            else:
                center_x = g_wall.rho * np.cos(g_wall.theta)
                center_y = g_wall.rho * np.sin(g_wall.theta)
                dist_to_robot = np.hypot(center_x - self.robot_x, center_y - self.robot_y)
                if dist_to_robot < self.config.sensor_range:
                    occluded = self.is_occluded(center_x, center_y, final_global_walls_data, final_global_cols_data)
                    if not occluded:
                        g_wall.misses += 1
                if g_wall.misses < self.config.max_misses_thresh or g_wall.hits >= 5:
                    walls_to_keep.append(g_wall)
                else:
                    self.get_logger().info(f'Removing wall ID {g_wall.id}.')
        self.global_walls = walls_to_keep
        
        columns_to_keep = []
        for g_column in self.global_columns:
            if g_column.id in matched_column_ids:
                g_column.misses = 0
                columns_to_keep.append(g_column)
            else:
                if np.hypot(g_column.x - self.robot_x, g_column.y - self.robot_y) > self.config.sensor_range:
                    g_column.misses = 0
                    columns_to_keep.append(g_column)
                else:
                    if np.hypot(g_column.x - self.robot_x, g_column.y - self.robot_y) < self.config.sensor_range:
                        if not self.is_occluded(g_column.x, g_column.y, final_global_walls_data, final_global_cols_data):
                            g_column.misses += 1
                    
                    if g_column.misses < self.config.max_misses_thresh:
                        columns_to_keep.append(g_column)
        self.global_columns = columns_to_keep

        if self.config.manhattan_opt_flag:
            self.optimizer.apply_global_topology(self.global_walls)

        # Publish global fusion result to G2O
        self.publish_to_g2o(msg.header, current_frame_wall_obs, current_frame_column_obs)

        # self.robot_trajectory.append((self.robot_x, self.robot_y))
        
        # Debug image drawing
        if self.config.publish_img_flag:
            with self.image_lock:
                import copy
                self.render_data = (
                    final_global_walls_data.copy(),
                    copy.deepcopy(self.global_walls),
                    copy.deepcopy(self.global_columns),
                    self.robot_x, self.robot_y,
                    list(self.active_debug_corners),
                    list(self.robot_trajectory)
                )
            self.new_image_event.set()

    def is_occluded(self, target_x, target_y, incoming_walls_data, incoming_columns_data):
        # Distance from sensor to target
        target_dist = np.hypot(target_x - self.robot_x, target_y - self.robot_y)
        # Check walls
        for row in incoming_walls_data:
            endpoints = polar2endpoints(row).flatten()
            x1, y1, x2, y2 = endpoints

            def ccw(A, B, C):
                return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
            
            A = np.array([self.robot_x, self.robot_y])
            B = np.array([target_x, target_y])
            C1 = np.array([x1, y1])
            C2 = np.array([x2, y2])
            if ccw(A, C1, C2) != ccw(B, C1, C2) and ccw(A, B, C1) != ccw(A, B, C2):
                return True
        
        # Check columns
        for row in incoming_columns_data:
            col_x, col_y, col_r = row
            col_dist = np.hypot(col_x, col_y)

            if col_dist > target_dist:
                continue

            # Check if the column intersects
            cross_prod = abs((col_x - self.robot_x)*(target_y - self.robot_y) - 
                             (col_y - self.robot_y)*(target_x - self.robot_x))
            if cross_prod / target_dist < (col_r * 1.2): # 1.2 is a safety margin
                return True
        return False
    
    def publish_to_g2o(self, header, current_frame_wall_obs: list, current_frame_column_obs: list):
        obs_array_msg = DetectionWithIDArray()
        obs_array_msg.header = header

        # wall -> Line2D
        for w_id, rel_wall, cov_matrix in current_frame_wall_obs:
            det = DetectionWithID()
            det.id = f"{w_id}"
            det.label = "wall"

            geom = Geometry()
            geom.type = Geometry.LINE
            
            line_msg = Line2D()
            
            # Local coordinates (cm -> m)
            local_rho = float(rel_wall[0])
            local_theta = float(rel_wall[1])
            
            line_msg.normal.x = math.cos(local_theta)
            line_msg.normal.y = math.sin(local_theta)
            line_msg.normal.z = 0.0
            line_msg.distance = local_rho / 100.0
            
            # Covariance (cm -> m)
            s_tt = float(cov_matrix[1, 1])             # (rad^2)
            s_td = float(cov_matrix[1, 0]) / 100.0     # (rad*m)
            s_dt = float(cov_matrix[0, 1]) / 100.0     # (m*rad)
            s_dd = float(cov_matrix[0, 0]) / 10000.0   # (m^2)
            line_msg.covariance = [s_tt, s_td, s_dt, s_dd]

            geom.line = line_msg
            det.geometry = geom
            obs_array_msg.detections.append(det)

        # Column -> Cylinder
        for c_id, rel_col in current_frame_column_obs:
            det = DetectionWithID()
            det.id = f"{c_id}" 
            det.label = "column"

            geom = Geometry()
            geom.type = Geometry.CYLINDER
            
            cyl_msg = Cylinder()
            
            # Center position (cm -> m)
            local_x = float(rel_col[0]) / 100.0
            local_y = float(rel_col[1]) / 100.0
            
            cyl_msg.pose = PoseWithCovariance()
            cyl_msg.pose.pose.position.x = local_x
            cyl_msg.pose.pose.position.y = local_y
            cyl_msg.pose.pose.position.z = 0.0
            
            # No rotation
            cyl_msg.pose.pose.orientation.w = 1.0
            cyl_msg.pose.pose.orientation.x = 0.0
            cyl_msg.pose.pose.orientation.y = 0.0
            cyl_msg.pose.pose.orientation.z = 0.0
            
            # 6x6 Matrix covariance for pose (x, y, z, roll, pitch, yaw)
            cov_6x6 = [0.0] * 36
            cov_6x6[0] = 0.0025
            cov_6x6[7] = 0.0025
            cov_6x6[14] = 1e-6
            cov_6x6[21] = 1e-6
            cov_6x6[28] = 1e-6
            cov_6x6[35] = 1e-6
            cyl_msg.pose.covariance = cov_6x6

            # Set radius and height
            local_r = float(rel_col[2]) / 100.0
            cyl_msg.radius = local_r
            cyl_msg.height = 2.0
            
            # Radius variance (assuming error 2cm -> 0.02^2 = 0.0004)
            cyl_msg.dimension_covariance = [0.0004, 0.0, 0.0, 0.01]
            
            geom.cylinder = cyl_msg
            det.geometry = geom
            obs_array_msg.detections.append(det)

        self.graph_opt_publisher.publish(obs_array_msg)

    def _do_debug_image_drawing(self, incoming_walls: np.ndarray, global_walls: list[GlobalWall], global_columns: list[GlobalColumn], rx, ry, corners, trajectory):
        img_size = self.config.processed_img_size
        
        all_x = [rx]
        all_y = [ry]

        for tx, ty in trajectory:
            all_x.append(tx)
            all_y.append(ty)
        
        for g_wall in global_walls:
            if g_wall.hits < 2: continue
            all_x.extend([g_wall.endpoints[0], g_wall.endpoints[2]])
            all_y.extend([g_wall.endpoints[1], g_wall.endpoints[3]])
            
        for g_column in global_columns:
            if g_column.hits < 2: continue
            all_x.extend([g_column.x - g_column.radius, g_column.x + g_column.radius])
            all_y.extend([g_column.y - g_column.radius, g_column.y + g_column.radius])

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        padding = 300.0 
        min_x -= padding; max_x += padding
        min_y -= padding; max_y += padding
        
        real_width = max_x - min_x
        real_height = max_y - min_y
        real_span = max(real_width, real_height)
        
        if real_span < 1000.0: 
            real_span = 1000.0
            
        scale = img_size / real_span
        center_x_real = (min_x + max_x) / 2.0
        center_y_real = (min_y + max_y) / 2.0
        center_img = img_size / 2.0
        
        def world_to_img(x, y):
            img_x = int((x - center_x_real) * scale + center_img)
            img_y = int(center_img - (y - center_y_real) * scale)
            return (img_x, img_y)

        canvas = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

        if len(trajectory) > 1:
            traj_img_pts = [world_to_img(tx, ty) for tx, ty in trajectory]
            traj_pts_np = np.array(traj_img_pts, np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [traj_pts_np], isClosed=False, color=(0, 165, 255), thickness=2)
        
        for row in incoming_walls:
            pts = polar2endpoints(row).flatten()
            p1 = world_to_img(pts[0], pts[1])
            p2 = world_to_img(pts[2], pts[3])
            cv2.line(canvas, p1, p2, (255, 0, 0), 5)

        for g_wall in global_walls:
            if g_wall.hits < 2: continue
            p1 = world_to_img(g_wall.endpoints[0], g_wall.endpoints[1])
            p2 = world_to_img(g_wall.endpoints[2], g_wall.endpoints[3])
            
            cv2.line(canvas, p1, p2, (0, 0, 255), 3)
            
            text_x = int((p1[0] + p2[0]) / 2)
            text_y = int((p1[1] + p2[1]) / 2) - 5
            cv2.putText(canvas, f"ID:{g_wall.id}", (text_x, text_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)
        
        for g_column in global_columns:
            if g_column.hits < 2: continue
            center = world_to_img(g_column.x, g_column.y)
            radius_img = int(g_column.radius * scale)
            
            cv2.circle(canvas, center, radius_img, (0, 255, 0), 2)
            cv2.putText(canvas, f"C:{g_column.id}", (center[0] + 5, center[1] - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 0), 1)

        robot_img_pos = world_to_img(self.robot_x, self.robot_y)
        cv2.circle(canvas, robot_img_pos, max(3, int(20 * scale)), (0, 0, 0), -1)

        for cx, cy in self.active_debug_corners:
            corner_img_pt = world_to_img(cx, cy)
            cv2.drawMarker(canvas, corner_img_pt, (255, 0, 255), markerType=cv2.MARKER_DIAMOND, markerSize=15, thickness=2)
            cv2.putText(canvas, "Corner", (corner_img_pt[0] + 8, corner_img_pt[1] - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        fused_img = Image()
        fused_img.header.stamp = self.get_clock().now().to_msg()
        fused_img.header.frame_id = 'lidar_frame'
        fused_img.height, fused_img.width = img_size, img_size
        fused_img.encoding, fused_img.step = 'bgr8', img_size * 3
        fused_img.data = canvas.tobytes()
        
        return fused_img

    def laser_to_img(self, vertices_laser: np.ndarray) -> np.ndarray:
        x_img = vertices_laser[0] + self.config.sensor_posx
        y_img = self.config.sensor_posy - vertices_laser[1]

        return np.column_stack((x_img, y_img)) 

def main(args=None):
    rclpy.init(args=args)
    global_fusion_node = GlobalFusionNode()
    rclpy.spin(global_fusion_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
