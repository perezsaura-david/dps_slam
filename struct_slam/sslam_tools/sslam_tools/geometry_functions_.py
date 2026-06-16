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


"""
Geometry functions for sslam_tools.
"""

import numpy as np

def endpoints2polar(endpoints: np.ndarray) -> np.ndarray:
    """
    Convert wall endpoints to polar coordinates (d, alpha, d1, d2).
    """
    # Safe check
    endpoints = endpoints.flatten()
    if endpoints.shape != (4,):
        raise ValueError("endpoints must be of shape (1,4)")
    polar_coords = []
    x1, y1, x2, y2 = endpoints.flatten()
    A = y1 - y2
    B = x2 - x1
    C = x1 * y2 - x2 * y1
    denominator = np.hypot(A, B)

    if denominator <= 1e-6: # points too close
        d = np.hypot(x1, y1)
        alpha = np.arctan2(y1, x1) % (2 * np.pi)
        return np.array([d, alpha, 0.0, 0.0])
    
    d = abs(C) / denominator
    if C > 0:
        nx, ny = -A, -B
    else:
        nx, ny = A, B
    alpha = np.arctan2(ny, nx) % (2 * np.pi)
    lx = -np.sin(alpha)
    ly = np.cos(alpha)
    d1 = x1 * lx + y1 * ly
    d2 = x2 * lx + y2 * ly  

    return np.array([d, alpha, d1, d2])

def polar2endpoints(polar: np.ndarray) -> np.ndarray:
    """
    Convert polar coordinates (d, alpha, d1, d2) to wall endpoints.
    """
    # Safe check
    polar = polar.flatten()
    if polar.shape != (4,):
        raise ValueError("polar must be of shape (1,4)")
    d, alpha, d1, d2 = polar.flatten()
    lx = -np.sin(alpha)
    ly = np.cos(alpha)
    nx = ly
    ny = -lx
    x0 = d * nx
    y0 = d * ny
    x1 = x0 + d1 * lx
    y1 = y0 + d1 * ly
    x2 = x0 + d2 * lx
    y2 = y0 + d2 * ly

    return np.array([x1, y1, x2, y2])

def calculate_line_intersection(d1, alpha1, d2, alpha2) -> np.ndarray:
    """
    Calculate the intersection point of two lines defined by polar coordinates.
    """
    det = np.sin(alpha2 - alpha1)
    if abs(det) < 1e-3:
        return None  # Lines are parallel or nearly parallel
    # Cramer's rule
    x = (d1 * np.sin(alpha2) - d2 * np.sin(alpha1)) / det
    y = (d2 * np.cos(alpha1) - d1 * np.cos(alpha2)) / det
    return np.array([x, y])

def point_to_segment_distance(px, py, x1, y1, x2, y2) -> float:
    """
    Calculate the minimum distance from a point to a line segment.
    """
    line_mag = np.hypot(x2 - x1, y2 - y1)

    if line_mag < 1e-6:
        return np.hypot(px - x1, py - y1)

    u = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag ** 2)

    if u < 0:
        ix, iy = x1, y1
    elif u > 1:
        ix, iy = x2, y2
    else:
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)

    return np.hypot(px - ix, py - iy)

def distance_point_to_line(point: np.ndarray, line_point1: np.ndarray, line_point2: np.ndarray) -> float:
    """Calculate the distance from a point to a line defined by two points."""
    line_vec = line_point2 - line_point1
    point_vec = point - line_point1
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-8:
        return np.linalg.norm(point_vec)
    line_unitvec = line_vec / line_len
    t = np.dot(point_vec, line_unitvec)
    nearest = line_point1 + t * line_unitvec
    dist = np.linalg.norm(point - nearest)
    return dist

def distance_point_to_line_segment(point: np.ndarray, line_point1: np.ndarray, line_point2: np.ndarray) -> float:
    """Calculate the distance from a point to a line segment."""
    dis1 = np.linalg.norm(point - line_point1)
    dis2 = np.linalg.norm(point - line_point2)
    dist = min(dis1, dis2)
    return dist

def overlap_cal(w1: np.ndarray, w2: np.ndarray) -> float:
    """Calculate the overlap length between two walls."""
    x1_1, y1_1, x2_1, y2_1 = w1
    x1_2, y1_2, x2_2, y2_2 = w2
    P1 = np.array([x1_1, y1_1])
    P2 = np.array([x2_1, y2_1])
    Q1 = np.array([x1_2, y1_2])
    Q2 = np.array([x2_2, y2_2])
    # Overlap calculation
    g_theta = endpoints2polar(w2)[1]
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
    return delta_distance

def intersection_cal_based_on_polar(rho1, theta1, rho2, theta2) -> np.ndarray:
    A = np.array([[np.cos(theta1), np.sin(theta1)],
                  [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    if abs(np.linalg.det(A)) < 1e-3:
        return None  # Lines are parallel or nearly parallel
    intersection = np.linalg.solve(A, b)
    return intersection.flatten() # x, y

def angle_logistic_function(delta_angle: float, angle_threshold: float, k: float = 20.0, m: float = 2.0) -> float:
    return m / (1.0 + np.exp(-k * (delta_angle - angle_threshold)))

def check_longitudinal_overlap(line_a: dict, line_b: dict, overlap_thresh: float = 0.0) -> bool:
    vec_a = line_a['p2'] - line_a['p1']
    norm_a = np.linalg.norm(vec_a)
    if norm_a < 1e-6: return False
    dir_a = vec_a / norm_a
    range_a = [0.0, norm_a]
    proj_b1 = np.dot(line_b['p1'] - line_a['p1'], dir_a)
    proj_b2 = np.dot(line_b['p2'] - line_a['p1'], dir_a)
    range_b = sorted([proj_b1, proj_b2])
    overlap = min(range_a[1], range_b[1]) - max(range_a[0], range_b[0])
    return overlap > overlap_thresh

def merge_wall_envelopes(segments: np.ndarray, thickness_thresh: float = 30.0,
                         angle_thresh_deg: float = 10.0, overlap_thresh:float =0.0) -> np.ndarray:
    """Skeletonization for LSD and Hought.| segment: Nx4 array - [x1, y1, x2, y2]"""
    if segments is None or len(segments) == 0:
        return np.array([])
    angle_thresh = np.deg2rad(angle_thresh_deg)
    active_mask = np.ones(len(segments), dtype=bool)
    lines_data = []
    for i, seg in enumerate(segments):
        x1, y1, x2, y2 = seg
        rho, theta,_,_ = endpoints2polar(seg)
        length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        lines_data.append({
            'id': i,
            'p1': np.array([x1, y1]),
            'p2': np.array([x2, y2]),
            'rho': rho,
            'theta': theta,
            'length': length
        })

    has_merged = True
    while has_merged:
        has_merged = False
        indices = np.where(active_mask)[0]
        n = len(indices)
        for i in range(n):
            idx_a = indices[i]
            if not active_mask[idx_a]: continue
            line_a = lines_data[idx_a]
            best_merge_idx = -1
            min_dist = float('inf')
            for j in range(i + 1, n):
                idx_b = indices[j]
                if not active_mask[idx_b]: continue
                line_b = lines_data[idx_b]
                # Angle check
                delta_angle = abs(line_a['theta'] - line_b['theta'])
                delta_angle = min(delta_angle, 2 * np.pi - delta_angle)
                if delta_angle > angle_thresh:
                    continue
                # Distance check
                mid_b = (line_b['p1'] + line_b['p2']) / 2
                dist = abs(distance_point_to_line(mid_b, line_a['p1'], line_a['p2']))
                if dist > thickness_thresh: continue
                # Overlap check
                if not check_longitudinal_overlap(line_a, line_b, overlap_thresh):
                    continue
                if dist < min_dist:
                    min_dist = dist
                    best_merge_idx = idx_b
            if best_merge_idx != -1:
                idx_b = best_merge_idx
                line_b = lines_data[idx_b]
                # Merge lines
                w_a = line_a['length']
                w_b = line_b['length']
                total_length = w_a + w_b
                new_line_angle = (line_a['theta'] * w_a + line_b['theta'] * w_b) / total_length
                dir_vec = np.array([-np.sin(new_line_angle), np.cos(new_line_angle)])
                all_points = np.vstack((line_a['p1'], line_a['p2'], line_b['p1'], line_b['p2']))
                centroid = np.average(all_points, axis=0, weights=[1,1,1,1])
                vecs = all_points - centroid
                scalars = np.dot(vecs, dir_vec)
                min_scalar = np.min(scalars)
                max_scalar = np.max(scalars)
                new_p1 = centroid + min_scalar * dir_vec
                new_p2 = centroid + max_scalar * dir_vec
                # Update line_a
                new_len = np.linalg.norm(new_p2 - new_p1)
                new_theta_normal = new_line_angle + np.pi / 2
                new_rho = new_p1[0] * np.cos(new_theta_normal) + new_p1[1] * np.sin(new_theta_normal)
                lines_data[idx_a] = {
                    'id': idx_a,
                    'p1': new_p1,
                    'p2': new_p2,
                    'rho': new_rho,
                    'theta': new_line_angle % (2 * np.pi),
                    'length': new_len
                }
                active_mask[idx_b] = False
                has_merged = True
                break
    result_segments = []
    for i in range(len(segments)):
        if active_mask[i]:
            line = lines_data[i]
            result_segments.append([line['p1'][0], line['p1'][1], line['p2'][0], line['p2'][1]])

    return np.array(result_segments)

        