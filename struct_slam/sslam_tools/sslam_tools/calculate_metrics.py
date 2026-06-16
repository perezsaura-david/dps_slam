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

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.geometry_functions_ import distance_point_to_line, endpoints2polar, polar2endpoints

def calculate_map_metrics(pred_lines: np.ndarray, gt_lines: np.ndarray, 
                          dist_thresh=500.0, angle_thresh_deg=5.0, angle_weight=50.0,
                          sample_step=1.0):
    # Recall, Precision and Geometric Error
    # Match predicted lines to ground truth lines
    pred_lines_match_gt_id = [-1] * len(pred_lines)  # -1 means unmatched
    angle_thresh_rad = np.deg2rad(angle_thresh_deg)
    for i, pred_line in enumerate(pred_lines):
        pred_line = np.array(pred_line)
        best_match_idx = -1
        best_match_score = float('inf')
        for gt_idx, gt_line in enumerate(gt_lines):
            gt_line = np.array(gt_line)
            delta_dis, delta_angle = diff_calc(pred_line, gt_line)
            if delta_dis < dist_thresh and delta_angle < angle_thresh_rad:
                match_score = delta_dis + angle_weight * delta_angle
                if match_score < best_match_score:
                    best_match_score = match_score
                    best_match_idx = gt_idx
        pred_lines_match_gt_id[i] = best_match_idx

    # Recall Calculation
    # Project to the matched gt lines and calculate the total matched length
    total_matched_length = 0.0
    matched_seg = [] # For visualization: x1, y1, x2, y2
    for gt_idx in range(len(gt_lines)):
        gt_line = np.array(gt_lines[gt_idx])
        gt_d, gt_a, _, _ = endpoints2polar(gt_line)
        matched_line = []
        for pred_idx, matched_gt_id in enumerate(pred_lines_match_gt_id):
            if matched_gt_id == gt_idx:
                pred_line = np.array(pred_lines[pred_idx])
                intersection_line = get_intersection_line(pred_line, gt_line)
                if np.linalg.norm(intersection_line) > 1e-6:
                    matched_line.append(intersection_line)
        # Calculate the total length of the matched segments on the gt line
        if len(matched_line) > 0:
            segments_1d = []
            t_vec = np.array([-np.sin(gt_a), np.cos(gt_a)])
            for seg in matched_line:
                p1 = np.array([seg[0], seg[1]])
                p2 = np.array([seg[2], seg[3]])
                d1 = np.dot(p1, t_vec)
                d2 = np.dot(p2, t_vec)
                segments_1d.append(tuple(sorted((d1, d2))))
            matched_length, merged_segments = merge_1d_segments(segments_1d)
            total_matched_length += matched_length
            for seg in merged_segments:
                x1, y1, x2, y2 = polar2endpoints(np.array([gt_d, gt_a, seg[0], seg[1]]))
                matched_seg.append((x1, y1, x2, y2))
    recall = total_matched_length / total_length_cal(gt_lines.tolist())

    # Precision Calculation
    # Project the predicted line to the matched gt line and find the matched part
    pred_lines_true_matched = []
    matched_seg_pred = [] # For visualization: x1, y1, x2, y2
    for pred_idx, matched_gt_id in enumerate(pred_lines_match_gt_id):
        if matched_gt_id != -1:
            pred_line = np.array(pred_lines[pred_idx])
            gt_line = np.array(gt_lines[matched_gt_id])
            true_matched_line = true_matched_pred_line(pred_line, gt_line)
            if np.linalg.norm(true_matched_line) > 1e-6:
                pred_lines_true_matched.append(true_matched_line)
                matched_seg_pred.append(true_matched_line)
            else:
                pred_lines_true_matched.append(np.array([0,0,0,0])) # Too short
        else:
            pred_lines_true_matched.append(np.array([0,0,0,0])) # No match
    total_true_matched_length = total_length_cal(pred_lines_true_matched)
    precision = total_true_matched_length / total_length_cal(pred_lines.tolist())

    # Geometric Error
    sq_error_sum = 0.0
    num_samples = 0
    for pred_idx, matched_gt_id in enumerate(pred_lines_match_gt_id):
        if matched_gt_id != -1:
            pred_line_true_matched = pred_lines_true_matched[pred_idx]
            gt_line = np.array(gt_lines[matched_gt_id])
            # Discretize the line segments and calculate the RMSE
            p1 = pred_line_true_matched[:2]
            p2 = pred_line_true_matched[2:]
            seg_len = np.linalg.norm(p2 - p1)
            if seg_len < 1e-6:
                continue
            num_samples = max(int(np.ceil(seg_len / sample_step)), 2)
            t_values = np.linspace(0, 1, num_samples)
            gt_p1 = gt_line[:2]
            gt_p2 = gt_line[2:]
            for t in t_values:
                pt = p1 + t * (p2 - p1)
                dist = distance_point_to_line(pt, gt_p1, gt_p2)
                sq_error_sum += dist ** 2
                num_samples += 1
    if num_samples > 0:
        geometric_error = np.sqrt(sq_error_sum / num_samples)
    else:
        geometric_error = 0.0

    dict_metrics = {
        "Recall": recall,
        "Precision": precision,
        "Geometric_Error": geometric_error
    }
    return dict_metrics, matched_seg, matched_seg_pred

def diff_calc(pred_wall: np.ndarray, gt_wall: np.array) -> float:
    # Calculate the distance and angle difference between two walls
    # pred_wall / gt_wall: [x1, y1, x2, y2]
    x1_1, y1_1, x2_1, y2_1 = pred_wall
    x1_2, y1_2, x2_2, y2_2 = gt_wall
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
    diff = abs(incli_new - incli_g)
    diff = diff % np.pi
    if diff > np.pi / 2:
        diff = np.pi - diff
    delta_angle = diff
    return delta_dis, delta_angle

def get_intersection_line(pre_line: np.ndarray, gt_line: np.ndarray) -> np.ndarray:
    # Project the predicted line to the ground truth line
    px1, py1, px2, py2 = pre_line
    gx1, gy1, gx2, gy2 = gt_line
    gt_vec = np.array([gx2 - gx1, gy2 - gy1])
    gt_len = np.linalg.norm(gt_vec)
    if gt_len < 1e-6:
        return np.array([0,0,0,0])
    gt_unit_vec = gt_vec / gt_len
    pred_vec_1 = np.array([px1 - gx1, py1 - gy1])
    pred_vec_2 = np.array([px2 - gx1, py2 - gy1])
    proj_len_1 = np.dot(pred_vec_1, gt_unit_vec)
    proj_len_2 = np.dot(pred_vec_2, gt_unit_vec)
    proj_pt_1 = np.array([gx1, gy1]) + proj_len_1 * gt_unit_vec
    proj_pt_2 = np.array([gx1, gy1]) + proj_len_2 * gt_unit_vec
    proj_line = np.array([proj_pt_1[0], proj_pt_1[1], proj_pt_2[0], proj_pt_2[1]])
    # Calculate the intersection by polar coordinates
    d, angle, gt_d1, gt_d2 = endpoints2polar(gt_line)
    _, _, proj_d1, proj_d2 = endpoints2polar(proj_line)
    gt_d1, gt_d2 = sorted([gt_d1, gt_d2])
    proj_d1, proj_d2 = sorted([proj_d1, proj_d2])
    match_d1 = max(gt_d1, proj_d1)
    match_d2 = min(gt_d2, proj_d2)
    if match_d1 < match_d2:
        match_line = polar2endpoints(np.array([d, angle, match_d1, match_d2]))
        return match_line
    else:
        return np.array([0,0,0,0])

def get_union_line(line1: np.ndarray, line2: np.ndarray) -> np.ndarray:
    # Check if the two lines are collinear
    union_line = []
    d1, angle1, d1_1, d1_2 = endpoints2polar(line1)
    d2, angle2, d2_1, d2_2 = endpoints2polar(line2)
    if abs(d1 - d2) > 1e-6 or abs(angle1 - angle2) > 1e-6:
        return union_line  # Not collinear -- 0
    # Get the union of the two lines in polar coordinates
    d = (d1 + d2) / 2
    angle = (angle1 + angle2) / 2
    d1_1, d1_2 = sorted([d1_1, d1_2])
    d2_1, d2_2 = sorted([d2_1, d2_2])
    if d1_2 < d2_1 or d2_2 < d1_1:
        union_line.append(line1.tolist())
        union_line.append(line2.tolist())
        return union_line  # No overlap, return both lines -- 2
    else:
        union_d1 = min(d1_1, d2_1)
        union_d2 = max(d1_2, d2_2)
        union_line = polar2endpoints(d, angle, union_d1, union_d2)
        return union_line # Overlap, return the union line -- 1
    
def merge_1d_segments(segments):
    if not segments:
        return 0.0
    segments.sort(key=lambda x: x[0])
    merged = []
    if segments:
        curr_start, curr_end = segments[0]
        for next_start, next_end in segments[1:]:
            if next_start < curr_end:
                curr_end = max(curr_end, next_end)
            else:
                merged.append((curr_start, curr_end))
                curr_start, curr_end = next_start, next_end
        merged.append((curr_start, curr_end))
    total_len = sum(end - start for start, end in merged)
    return total_len, merged

def true_matched_pred_line(pred_line: np.ndarray, gt_line: np.ndarray) -> np.ndarray:
    intersection_line = get_intersection_line(pred_line, gt_line)
    if np.linalg.norm(intersection_line) < 1e-6:
        return np.array([0,0,0,0]) # No intersection
    else:
        # Return the matched part of the predicted line
        p1 = pred_line[:2]
        p2 = pred_line[2:]
        i1 = intersection_line[:2]
        i2 = intersection_line[2:]
        vec1 = i2 - i1
        perp_vec = np.array([-vec1[1], vec1[0]])
        vec2 = p2 - p1
        denom = np.cross(vec2, perp_vec)
        if abs(denom) < 1e-6: # Lines are perpendicular
            return np.array([0,0,0,0])
        u1 = np.cross(i1 - p1, vec2) / denom
        inter_p1 = i1 + u1 * perp_vec
        u2 = np.cross(i2 - p1, vec2) / denom
        inter_p2 = i2 + u2 * perp_vec
        return np.array([inter_p1[0], inter_p1[1], inter_p2[0], inter_p2[1]])

def total_length_cal(lines: list) -> float:
    total_length = 0.0
    for line in lines:
        pt1 = [line[0], line[1]]
        pt2 = [line[2], line[3]]
        total_length += np.linalg.norm(np.array(pt1) - np.array(pt2))
    return total_length