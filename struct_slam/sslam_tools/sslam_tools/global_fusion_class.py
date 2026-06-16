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


import numpy as np
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.geometry_functions_ import polar2endpoints, intersection_cal_based_on_polar

class GlobalWall:
    def __init__(self, id, rho, theta, d1, d2):
        self.id = id
        # Geometric parameters
        self.rho = rho
        self.theta = theta
        self.d1 = d1
        self.d2 = d2
        self.endpoints: np.ndarray = polar2endpoints(np.array([rho, theta, d1, d2])) # x1, y1, x2, y2
        # Observation parameters
        self.hits = 1
        self.misses = 0
        # Variance parameters
        range_var = 100.0 # cm^2
        angle_var = np.deg2rad(10.0) ** 2 # rad^2
        self.covariance_state = np.array([[range_var, 0], 
                                          [0, angle_var]]) # state covariance
        self.covariance_mea = np.array([[range_var, 0], 
                                        [0, angle_var]])  # measurement covariance
        # self.covariance_state = np.array([[range_var, 0, 0, 0], 
        #                                   [0, angle_var, 0, 0],
        #                                   [0, 0, range_var, 0],
        #                                   [0, 0, 0, range_var]]) # state covariance
        # self.covariance_mea = np.array([[range_var, 0, 0, 0], 
        #                                 [0, angle_var, 0, 0],
        #                                 [0, 0, range_var, 0],
        #                                 [0, 0, 0, range_var]])  # measurement covariance

    def update(self, rho_mea, theta_mea):
        # Ommit prediction step
        # Measurement update
        K = self.covariance_state @ np.linalg.inv(self.covariance_state + self.covariance_mea)
        diff_rho = rho_mea - self.rho
        diff_theta = theta_mea - self.theta
        diff_theta = (diff_theta + np.pi) % (2 * np.pi) - np.pi
        diff_vector = np.array([[diff_rho], [diff_theta]])

        state_vector = np.array([[self.rho], [self.theta]])
        updated_state = state_vector + K @ diff_vector
        self.rho = updated_state[0, 0]
        self.theta = updated_state[1, 0] % (2 * np.pi)
        # Update covariance
        self.covariance_state = (np.eye(2) - K) @ self.covariance_state
        # Update observation parameters
        self.hits += 1
        self.misses = 0

    def recalculate_endpoints_on_line(self, new_raw_endpoints: np.ndarray):
        # Recalculate endpoints on the updated line
        dir_vec = np.array([-np.sin(self.theta), np.cos(self.theta)])
        center_pt = np.array([self.rho * np.cos(self.theta), self.rho * np.sin(self.theta)])

        pts = [
            self.endpoints[0:2],
            self.endpoints[2:4],
            new_raw_endpoints[0:2],
            new_raw_endpoints[2:4]
        ]

        projections = [np.dot(pt - center_pt, dir_vec) for pt in pts]

        current_d1, current_d2 = self.d1, self.d2
        min_proj = max(np.min(projections), current_d1 - 50.0)
        max_proj = min(np.max(projections), current_d2 + 50.0)

        self.d1 = min_proj
        self.d2 = max_proj

        p_start = center_pt + self.d1 * dir_vec
        p_end = center_pt + self.d2 * dir_vec
        # Update endpoints and d1, d2
        self.endpoints = np.array([p_start[0], p_start[1], p_end[0], p_end[1]])

    def recalculate_d_parameters(self):
        # Recalculate d1 and d2 based on current endpoints
        dir_vec = np.array([-np.sin(self.theta), np.cos(self.theta)])
        center_pt = np.array([self.rho * np.cos(self.theta), self.rho * np.sin(self.theta)])
        p_start = self.endpoints[0:2]
        p_end = self.endpoints[2:4]
        vec_start = p_start - center_pt
        vec_end = p_end - center_pt
        self.d1 = np.dot(vec_start, dir_vec)
        self.d2 = np.dot(vec_end, dir_vec)
        if self.d1 > self.d2:
            self.d1, self.d2 = self.d2, self.d1
            self.endpoints = np.concatenate((p_end, p_start))

class GlobalColumn:
    def __init__(self, id, x, y, radius):
        self.id = id
        self.x = x
        self.y = y
        self.radius = radius
        self.hits = 1
        self.misses = 0

    def update(self, x_mea, y_mea, radius_mea):
        # Simple averaging update
        self.x = (self.x * self.hits + x_mea) / (self.hits + 1)
        self.y = (self.y * self.hits + y_mea) / (self.hits + 1)
        self.radius = (self.radius * self.hits + radius_mea) / (self.hits + 1)
        self.hits += 1
        self.misses = 0

class ManhattanWorldOptimizer:
    def __init__(self):
        self.manhatten_theta_thresh = np.deg2rad(15.0) # Wall adsorption angle threshold
        self.corner_connect_dist_thresh = 10.0 # Corner closure threshold
        self.min_intersection_angle = np.deg2rad(85.0)
        self.max_intersection_angle = np.deg2rad(95.0)

    def apply_global_topology(self, global_walls: list[GlobalWall]):
        if not global_walls:
            return
        # Find the dominant direction
        dominant_theta = self.find_dominant_direction(global_walls)
        # Apply Manhattan world constraints
        self.apply_manhattan_constraints(global_walls, dominant_theta)
        # Adjust wall endpoints to close corners
        self.close_corners(global_walls)

    def find_dominant_direction(self, global_walls: list[GlobalWall]) -> float:
        mapped_angles = []
        weights = []
        for wall in global_walls:
            length = wall.d2 - wall.d1
            w = wall.hits * length
            norm_theta = wall.theta % (np.pi / 2)
            mapped_angles.append(norm_theta)
            weights.append(w)
        if not weights:
            return 0.0
        n_bins = 90
        bin_width = (np.pi / 2) / n_bins
        histogram = np.zeros(n_bins)
        for theta, w in zip(mapped_angles, weights):
            bin_idx = int(theta / bin_width) % n_bins
            histogram[bin_idx] += w
        smoothed_hist = np.zeros(n_bins)
        for i in range(n_bins):
            prev_idx = (i - 1 + n_bins) % n_bins
            next_idx = (i + 1) % n_bins
            
            smoothed_hist[i] = (0.25 * histogram[prev_idx] + 
                                0.50 * histogram[i] + 
                                0.25 * histogram[next_idx])
        peak_bin_idx = np.argmax(smoothed_hist)
        refine_sin = 0.0
        refine_cos = 0.0
        refine_weight = 0.0
        for theta, w in zip(mapped_angles, weights):
            bin_center = (peak_bin_idx + 0.5) * bin_width
            diff = theta - bin_center
            diff = (diff + np.pi/4) % (np.pi/2) - np.pi/4
            if abs(diff) < np.deg2rad(5.0):
                refine_sin += w * np.sin(4 * theta)
                refine_cos += w * np.cos(4 * theta)
                refine_weight += w
        if refine_weight > 0:
            avg_4_theta = np.arctan2(refine_sin, refine_cos)
            dominant_theta = (avg_4_theta / 4.0) % (np.pi / 2)
        else:
            dominant_theta = (peak_bin_idx + 0.5) * bin_width
        return dominant_theta
    
    def apply_manhattan_constraints(self, global_walls: list[GlobalWall], dominant_theta: float):
        for wall in global_walls:
            current_theta = wall.theta
            diff = (current_theta - dominant_theta) % (2 * np.pi)
            k = round(diff / (np.pi / 2))
            target_theta = (dominant_theta + k * (np.pi / 2)) % (2 * np.pi)
            actual_diff = abs(np.arctan2(np.sin(current_theta - target_theta), np.cos(current_theta - target_theta)))
            if actual_diff < self.manhatten_theta_thresh:
                pts = wall.endpoints
                mid_x = (pts[0] + pts[2]) / 2.0
                mid_y = (pts[1] + pts[3]) / 2.0
                new_theta = target_theta % (2 * np.pi)
                new_rho = mid_x * np.cos(new_theta) + mid_y * np.sin(new_theta)
                # Update wall parameters
                wall.theta = new_theta
                wall.rho = new_rho
                # Recalculate endpoints
                wall.recalculate_endpoints_on_line(wall.endpoints)

    def close_corners(self, global_walls: list[GlobalWall]):
        n = len(global_walls)
        for i in range(n):
            for j in range(i + 1, n):
                w1 = global_walls[i]
                w2 = global_walls[j]
                # Angle check
                angle_diff = abs(w1.theta - w2.theta)
                angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
                if angle_diff < self.min_intersection_angle or angle_diff > self.max_intersection_angle:
                    continue
                # Intersection distance check
                p1_start = np.array([w1.endpoints[0], w1.endpoints[1]])
                p1_end = np.array([w1.endpoints[2], w1.endpoints[3]])
                p2_start = np.array([w2.endpoints[0], w2.endpoints[1]])
                p2_end = np.array([w2.endpoints[2], w2.endpoints[3]])
                dists = [
                    (np.linalg.norm(p1_start - p2_start), 'ss'),
                    (np.linalg.norm(p1_start - p2_end), 'se'),
                    (np.linalg.norm(p1_end - p2_start), 'es'),
                    (np.linalg.norm(p1_end - p2_end), 'ee')
                ]
                min_dist, pair_type = min(dists, key=lambda x: x[0])
                if min_dist < self.corner_connect_dist_thresh:
                    intersection = intersection_cal_based_on_polar(w1.rho, w1.theta, w2.rho, w2.theta)
                    if intersection is not None:
                        ix, iy = intersection
                        if pair_type[0] == 's':
                            w1.endpoints[0], w1.endpoints[1] = ix, iy
                        else:
                            w1.endpoints[2], w1.endpoints[3] = ix, iy
                        w1.recalculate_d_parameters()
                        if pair_type[1] == 's':
                            w2.endpoints[0], w2.endpoints[1] = ix, iy
                        else:
                            w2.endpoints[2], w2.endpoints[3] = ix, iy
                        w2.recalculate_d_parameters()