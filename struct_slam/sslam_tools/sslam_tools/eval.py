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
from msg_interfaces.msg import Wall, GlobalFusionResult

import yaml
import numpy as np
import os
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
import cv2
import datetime
from ament_index_python.packages import get_package_share_directory

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.calculate_metrics import calculate_map_metrics

def lines_to_dense_points(lines, step=1.0):
    all_points = []
    if len(lines) == 0:
        return np.empty((0, 2))
    
    for line in lines:
        p1 = np.array([line[0], line[1]])
        p2 = np.array([line[2], line[3]])
        dist = np.linalg.norm(p2 - p1)
        num_points = max(2, int(np.ceil(dist / step)))
        xs = np.linspace(p1[0], p2[0], num_points)
        ys = np.linspace(p1[1], p2[1], num_points)
        all_points.append(np.column_stack((xs, ys)))

    return np.vstack(all_points)

def get_transform_matrix(x, y, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A -centroid_A
    BB = B -centroid_B

    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = np.dot(Vt.T, U.T)

    t = centroid_B - np.dot(R, centroid_A)

    T = np.identity(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T

def transform_lines(lines: np.ndarray, T: np.ndarray) -> np.ndarray:
    if len(lines) == 0: return lines
    transformed = []
    for line in lines:
        p1 = np.array([line[0], line[1], 1.0])
        p2 = np.array([line[2], line[3], 1.0])
        p1_new = T @ p1
        p2_new = T @ p2
        transformed.append([p1_new[0], p1_new[1], p2_new[0], p2_new[1]])
    return np.array(transformed, dtype=np.float32)

def icp_with_initial_guess(pred_lines: np.ndarray, gt_lines: np.ndarray,
                           init_x = 0.0, init_y = 0.0, init_theta = 0.0,
                           max_iterations=200, tolerance=1e-4, match_dist_thresh=100):
    src_points = lines_to_dense_points(pred_lines, step = 1)
    dst_points = lines_to_dense_points(gt_lines, step = 1)

    if src_points.shape[0] == 0 or dst_points.shape[0] == 0:
        return np.eye(3), src_points, float('inf')

    src = np.ones((src_points.shape[0], 3))
    src[:, :2] = src_points

    init_T = get_transform_matrix(init_x, init_y, init_theta)
    src = (init_T @ src.T).T
    tree = cKDTree(dst_points)
    prev_error = float('inf')
    cumulative_T = init_T

    for i in range(max_iterations):
        distances, indices = tree.query(src[:, :2])
        valid_mask = distances < match_dist_thresh

        if np.sum(valid_mask) < 10:
            print("Not enough valid matches, stopping ICP.", flush=True)
            break

        valid_src = src[valid_mask][:, :2]
        valid_dst = dst_points[indices[valid_mask]]

        mean_error = np.mean(distances[valid_mask])
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

        T_delta = best_fit_transform(valid_src, valid_dst)
        src[:, :2] = (T_delta[:2, :2] @ src[:, :2].T).T + T_delta[:2, 2]
        cumulative_T = T_delta @ cumulative_T

    return cumulative_T, src[:, :2], mean_error

class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        # Upload ground truth data
        # How to use: ros2 run sslam_tools eval_node --ros-args -p scenario:=garage_part
        self.declare_parameter('scenario', 'dungeon') # garage_part, cslab, corridor, dungeon 
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.get_logger().info(f"Target Scenario: {self.scenario}")
        try:
            package_share_path = get_package_share_directory('sslam_tools')
            self.gt_path = os.path.join(package_share_path, 'resource', 'ground_truth.yaml')
        except Exception as e:
            self.get_logger().error(f"Error finding package path: {e}")
            return
        self.gt_lines = self.load_gt_yaml(self.gt_path)
        if self.gt_lines is None or len(self.gt_lines) == 0:
            self.get_logger().warn(f"No GT lines found for scenario '{self.scenario}' in {self.gt_path}")
        else:
            self.get_logger().info(f"Successfully loaded {len(self.gt_lines)} lines for scenario '{self.scenario}'")

        # Subscribe to global fusion results
        self.global_fusion_subscription = self.create_subscription(
            GlobalFusionResult,
            '/global_fusion_result',
            self.listener_callback,
            10
        )

        # Config
        self.epoch = 0

        # Metrics List
        self.rmse_list = []
        self.Recall_list = []
        self.Precision_list = []
        self.Geometric_Error_list = []

        # Data Collection
        self.flag_output = False
        if self.flag_output:
            detector_name = "yolo" # yolo, lsd, hought, ransac
            ymdhms = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.data_path = f'data/{detector_name}/{self.scenario}_{detector_name}_metrics_{ymdhms}.csv'
            with open(self.data_path, 'w') as f:
                f.write('RMSE(ICP)_cm,Recall,Precision,Geometric_Error_cm,Distance_Error_cm,Angle_Error_Deg\n')

        # Visualization
        self.visualize_enable = True
        # img_size = 4096 + 2048
        img_size = 2048 + 1024
        visual_img_size = 640
        self.ratio = visual_img_size / img_size
        self.img = np.ones((visual_img_size, visual_img_size, 3), dtype=np.uint8) * 255
        self.sensor_posx = img_size // 2
        self.sensor_posy = img_size // 2

        # boundary flag
        if self.boundary_flag:
            self.boundary = {"x_left": -400.0,
                             "x_right": 400.0,
                             "y_top": 400.0,
                             "y_bottom": -400.0}

    def load_gt_yaml(self, path) -> np.ndarray:
        if not os.path.exists(path):
            self.get_logger().error(f"GT File not found: {path}")
            return None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            lines_list = []
            if 'contours' in data and data['contours']:
                for item in data['contours']:
                    if item.get('scenario') != self.scenario:
                        continue
                    # Match the scenario
                    points = item.get('points', [])
                    is_closed = item.get('closed', False)
                    if len(points) < 2:
                        continue
                    for i in range(len(points) - 1):
                        lines_list.append([
                            float(points[i][0]), float(points[i][1]), 
                            float(points[i+1][0]), float(points[i+1][1])
                        ])
                    # Close the contour
                    if is_closed:
                        lines_list.append([
                            float(points[-1][0]), float(points[-1][1]), 
                            float(points[0][0]), float(points[0][1])
                        ])
                    # Boundary flag
                    self.boundary_flag = item.get('boundary_flag', False)
                    # Get offset and rotation if available
                    offset = item.get('offset', {})
                    self.offsetX = offset.get('x', 0.0)
                    self.offsetY = offset.get('y', 0.0)
                    self.rotation = np.radians(offset.get('rotation', 0.0))
                    # Match distance threshold for ICP
                    self.match_dist_thresh = item.get('match_dist_thresh', 100.0)
            return np.array(lines_list, dtype=np.float32)
            
        except Exception as e:
            self.get_logger().error(f"Error parsing YAML: {e}")
            return None
        
    def listener_callback(self, msg):
        if msg is None or len(msg.wall_resluts) < 5:
            self.get_logger().warn("Received empty global fusion result.")
            return
        self.get_logger().info(f"=== Epoch {self.epoch} ===")
        self.get_logger().info(f"Received global fusion result with {len(msg.wall_resluts)} walls")
        self.epoch += 1
        # Match detected walls with GT lines
        # Ground Truth
        gt_lines = self.gt_lines
        # Valid Detected Walls -- Eleminate elements out of range
        detected_walls = []
        for wall in msg.wall_resluts:
            if self.boundary_flag:
                if not self.is_wall_within_boundary(wall):
                    continue
            detected_walls.append(wall.endpoints)
        detected_walls = np.array(detected_walls, dtype=np.float32)

        final_T, aligned_points, final_rmse = icp_with_initial_guess(
            detected_walls, 
            gt_lines, 
            init_x=self.offsetX, 
            init_y=self.offsetY, 
            init_theta=self.rotation,
            match_dist_thresh=self.match_dist_thresh
        )

        # Metrics calculation
        aligned_detected_walls = transform_lines(detected_walls, final_T)
        metrics, matched_seg, matched_seg_pred = calculate_map_metrics(aligned_detected_walls, gt_lines, dist_thresh = 30.0)
        # Store metrics
        self.rmse_list.append(final_rmse)
        self.Recall_list.append(metrics["Recall"])
        self.Precision_list.append(metrics["Precision"])
        self.Geometric_Error_list.append(metrics["Geometric_Error"])

        # log the results
        self.get_logger().info(f"RMSE (ICP): {final_rmse:.4f}")
        final_theta = np.arctan2(final_T[1, 0], final_T[0, 0])
        final_tx = final_T[0, 2]
        final_ty = final_T[1, 2]
        self.get_logger().info(f"Recovered Pose -> x: {final_tx:.2f}, y: {final_ty:.2f}, theta: {np.degrees(final_theta):.2f} deg")
        self.get_logger().info(f"Recall: {metrics['Recall']:.4f}")
        self.get_logger().info(f"Precision: {metrics['Precision']:.4f}")
        self.get_logger().info(f"Geometric Error: {metrics['Geometric_Error']:.4f} cm")
        self.get_logger().info(f"Distance Error: {metrics['Distance_Error']:.10f} cm")
        self.get_logger().info(f"Angle Error: {metrics['Angle_Error_Deg']:.10f} deg")

        if self.flag_output:
            with open(self.data_path, 'a') as f:
                f.write(f'{final_rmse:.4f},{metrics["Recall"]:.4f},{metrics["Precision"]:.4f},{metrics["Geometric_Error"]:.4f},{metrics["Distance_Error"]:.10f},{metrics["Angle_Error_Deg"]:.10f}\n')

        if self.visualize_enable:
            self.visualize(gt_lines, detected_walls, aligned_detected_walls, matched_seg, matched_seg_pred)
            # if self.epoch == 100:
            #     self.result_plt()
            #     plt.show()

    def is_wall_within_boundary(self, wall: Wall) -> bool:
        P1 = [wall.endpoints[0], wall.endpoints[1]]
        P2 = [wall.endpoints[2], wall.endpoints[3]]
        if (self.boundary["x_left"] <= P1[0] <= self.boundary["x_right"] and
            self.boundary["y_bottom"] <= P1[1] <= self.boundary["y_top"] and
            self.boundary["x_left"] <= P2[0] <= self.boundary["x_right"] and
            self.boundary["y_bottom"] <= P2[1] <= self.boundary["y_top"]):
            return True

        return False
    
    def result_plt(self):
        epochs = list(range(1, self.epoch + 1))
        plt.figure(figsize=(12, 8))
        # RMSE plot
        plt.subplot(2, 2, 1)
        plt.plot(epochs, self.rmse_list, marker='o')
        plt.title('ICP RMSE Over Epochs')
        plt.xlabel('Epoch')
        plt.ylabel('RMSE')
        plt.grid()

        # Recall plot
        plt.subplot(2, 2, 2)
        plt.plot(epochs, self.Recall_list, marker='o')
        plt.title('Recall Over Epochs')
        plt.xlabel('Epoch')
        plt.ylabel('Recall')
        plt.grid()

        # Precision plot
        plt.subplot(2, 2, 3)
        plt.plot(epochs, self.Precision_list, marker='o')
        plt.title('Precision Over Epochs')
        plt.xlabel('Epoch')
        plt.ylabel('Precision')
        plt.grid()

        # Geometric Error plot
        plt.subplot(2, 2, 4)
        plt.plot(epochs, self.Geometric_Error_list, marker='o')
        plt.title('Geometric Error Over Epochs')
        plt.xlabel('Epoch')
        plt.ylabel('Geometric Error')
        plt.grid()

    def visualize(self, gt_lines, detected_walls, aligned_detected_walls, matched_seg=None, matched_seg_pred=None):
        visual_img = self.img.copy()

        COLOR_GT = (34, 139, 34)       # ForestGreen
        COLOR_ALIGNED = (235, 206, 135)# SkyBlue
        COLOR_MATCH_GT = (0, 165, 255) # Orange
        COLOR_MATCH_PRED = (180, 105, 255) # Violet
        
        # GT: ForestGreen
        for line in gt_lines:
            pt1 = self.laser_to_img(line[0], line[1]) * self.ratio
            pt2 = self.laser_to_img(line[2], line[3]) * self.ratio
            cv2.line(visual_img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), COLOR_GT, 2)
        # Detected: red
        # for wall in detected_walls:
        #     pt1 = self.laser_to_img(wall[0], wall[1]) * self.ratio
        #     pt2 = self.laser_to_img(wall[2], wall[3]) * self.ratio
        #     cv2.line(visual_img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 0, 255), 2)
        # Aligned points: SkyBlue
        if aligned_detected_walls is not None and len(aligned_detected_walls) > 0:
            for wall in aligned_detected_walls:
                pt1 = self.laser_to_img(wall[0], wall[1]) * self.ratio
                pt2 = self.laser_to_img(wall[2], wall[3]) * self.ratio
                cv2.line(visual_img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), COLOR_ALIGNED, 2)
        # Matched segments on ground truth: Orange
        if matched_seg is not None:
            for seg in matched_seg:
                pt1 = self.laser_to_img(seg[0], seg[1]) * self.ratio
                pt2 = self.laser_to_img(seg[2], seg[3]) * self.ratio
                cv2.line(visual_img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), COLOR_MATCH_GT, 2)
        # Matched segments on prediction: Violet
        if matched_seg_pred is not None:
            for seg in matched_seg_pred:
                pt1 = self.laser_to_img(seg[0], seg[1]) * self.ratio
                pt2 = self.laser_to_img(seg[2], seg[3]) * self.ratio
                cv2.line(visual_img, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), COLOR_MATCH_PRED, 2)
        cv2.imshow("Alignment Visualization", visual_img)
        cv2.waitKey(1)

    def laser_to_img(self, x, y):
        img_x = self.sensor_posx + x
        img_y = self.sensor_posy - y
        return np.array([img_x, img_y])
        
def main(args=None):
    rclpy.init(args=args)
    evaluation_node = EvaluationNode()
    rclpy.spin(evaluation_node)
    evaluation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()