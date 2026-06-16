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
Some functions for geometric calculations.
"""

import numpy as np
import cv2

def calculate_walls_endpoints(wall_vertices):
    """Calculate wall endpoints from wall vertices."""
    distances = []

    for i in range(1, 4):
        dist = np.linalg.norm(wall_vertices[i] - wall_vertices[0])
        distances.append(dist)

    min_index = 0
    max_index = 0
    min_val = distances[0]
    max_val = distances[0]

    for i, val in enumerate(distances):
        if val < min_val:
            min_val = val
            min_index = i
        if val > max_val:
            max_val = val
            max_index = i

    # Adjust index since distances start from vertex 1
    min_index += 1
    max_index += 1
    mid_index = 6 - min_index - max_index
    midpoint1 = (wall_vertices[0] + wall_vertices[min_index]) / 2
    midpoint2 = (wall_vertices[mid_index] + wall_vertices[max_index]) / 2
    
    # Calculate slope
    # delta_y = midpoint2[1] - midpoint1[1]
    # delta_x = midpoint2[0] - midpoint1[0]
    # slope = delta_y / delta_x if delta_x != 0 else float('inf')
    
    return midpoint1, midpoint2

def calculate_columns_center_radius(column_vertices):
    """Calculate column center and radius from column vertices."""
    distances = []

    for i in range(1, 4):
        dist = np.linalg.norm(column_vertices[i] - column_vertices[0])
        distances.append(dist)

    min_index = 0
    max_index = 0
    min_val = distances[0]
    max_val = distances[0]

    for i, val in enumerate(distances):
        if val < min_val:
            min_val = val
            min_index = i
        if val > max_val:
            max_val = val
            max_index = i

    min_index += 1
    max_index += 1

    center = (column_vertices[0] + column_vertices[max_index]) / 2

    radius = (max_val) / 2

    return center, radius

def calculate_wall_angle(wall_endpoints):
    """
    Calculate the angle between the line connecting the endpoints and the origin and the x-axis.
    Range: 0 ~ 2 π
    """
    p1, p2 = wall_endpoints
    angle_1 = np.arctan2(p1[1], p1[0])
    angle_2 = np.arctan2(p2[1], p2[0])
    if angle_1 < 0:
        angle_1 += 2 * np.pi
    if angle_2 < 0:
        angle_2 += 2 * np.pi
    angle = np.array([angle_1, angle_2])
    return angle

def sort_wall_endpoints_by_angle(walls_endpoints_list):
    """Sort wall endpoints based on their angles with respect to the origin."""
    walls_with_angles = []
    for endpoints in walls_endpoints_list:
        angles = calculate_wall_angle(endpoints)
        walls_with_angles.append((endpoints, angles))
    
    # Sort by the minimum angle of each wall
    walls_with_angles.sort(key=lambda x: np.min(x[1]))
    
    sorted_walls_endpoints = [item[0] for item in walls_with_angles]
    return sorted_walls_endpoints

def is_same_wall(wall_a, wall_b, eps_angle = 0.2, eps_dist = 100):
    """Determine if two walls are the same based on angle and distance criteria."""

    def distance_point_to_line(point, line_point1, line_point2):
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

    vec_a = wall_a[1, :] - wall_a[0, :]
    vec_b = wall_b[1, :] - wall_b[0, :]
    vec_cen = (wall_a[1, :] + wall_a[0, :])/2 - (wall_b[1, :] + wall_b[0, :])/2
    # vec_emerge = (vec_a + vec_b)/2 # don't use this, because it may lead to zero vector
    vec_emerge = vec_a

    # self.get_logger().info(f'Vec_a: {vec_a}, Vec_b: {vec_b}, Vec_cen: {vec_cen}, Vec_emerge: {vec_emerge}')

    dis_a_12wallb = distance_point_to_line(wall_a[0, :], wall_b[0, :], wall_b[1, :])
    dis_a_22wallb = distance_point_to_line(wall_a[1, :], wall_b[0, :], wall_b[1, :])

    if  np.linalg.norm(vec_cen) < 1e-8:
        return np.bool_(True)
    
    cos_angle_a_b = np.abs(np.dot(vec_a, vec_b)) / (np.linalg.norm(vec_a) * np.linalg.norm(vec_b))
    cos_angle_cen_emerge = np.abs(np.dot(vec_cen, vec_emerge)) / (np.linalg.norm(vec_cen) * np.linalg.norm(vec_emerge))

    flag_angle = (cos_angle_cen_emerge > 1 - eps_angle) and (cos_angle_a_b > 1 - eps_angle)
    flag_dist = (dis_a_12wallb < eps_dist) or (dis_a_22wallb < eps_dist)

    # self.get_logger().info(f'{cos_angle_cen_emerge}, {cos_angle_a_b}, {dis_a_12wallb}, {dis_a_22wallb}')

    if flag_angle and flag_dist:
        return np.bool_(True)
    
    return np.bool_(False)

def no_intersection(endpoints_current, endpoints_next):
    """Determine the corner points between two walls that do not intersect."""
    # choose points as corners
    angle_current = calculate_wall_angle(endpoints_current)
    angle_next = calculate_wall_angle(endpoints_next)
    if angle_current[0] < angle_current[1]:
        point_current = endpoints_current[1,:]
    else:
        point_current = endpoints_current[0,:]
    if angle_next[0] < angle_next[1]:
        point_next = endpoints_next[0,:]
    else:
        point_next = endpoints_next[1,:]
    return point_current, point_next

def perpendicularfoot_distance_angle(wall_endpoints):
    """
    Calculate the foot point, perpendicular distance to origin, and angle of the wall.
    input: wall_endpoints: np.array of shape (2,2)
    """
    # Fit line: (x, y) = (x0, y0) + t * (v_x, v_y)
    p1, p2 = wall_endpoints
    delta = p2 - p1
    v_x, v_y = delta / np.linalg.norm(delta)
    x0, y0 = (p1 + p2) / 2
    t = -(v_x * (x0) + v_y * (y0)) / (v_x**2 + v_y**2)
    foot_x = x0 + t * v_x
    foot_y = y0 + t * v_y
    foot_point = np.array([foot_x, foot_y])
    d = np.linalg.norm(foot_point)
    if -v_y * x0 + v_x * y0 < 0:
        d = -d
    alpha = np.arctan2(v_y, v_x)  # -π ~ π
    if alpha < 0:
        alpha += np.pi  # Convert to 0 ~ π range
    return foot_point, d, alpha

def wall_endpoints2dadd(wall_endpoints):
    """Convert wall endpoints to DADD format."""
    P, d, angle = perpendicularfoot_distance_angle(wall_endpoints)
    v1 = wall_endpoints[0,:] - P
    v2 = wall_endpoints[1,:] - P
    d1 = np.linalg.norm(v1)
    d2 = np.linalg.norm(v2)

    if wall_endpoints[0,0] < P[0] or (wall_endpoints[0,0] == P[0] and wall_endpoints[0,1] < P[1]):
        d1 = -d1
    if wall_endpoints[1,0] < P[0] or (wall_endpoints[1,0] == P[0] and wall_endpoints[1,1] < P[1]):
        d2 = -d2

    return d, angle, d1, d2

def dadd2wall_endpoints(d, angle, d1, d2):
    """Convert DADD format to wall endpoints."""
    if angle > np.pi/2:
        foot_point = np.array([d * np.sin(angle), -d * np.cos(angle)])
        v_x = -np.cos(angle)
        v_y = -np.sin(angle)
    else:
        foot_point = np.array([-d * np.sin(angle), d * np.cos(angle)])
        v_x = np.cos(angle)
        v_y = np.sin(angle)

    line_dir = np.array([v_x, v_y])

    endpoint1 = foot_point + d1 * line_dir
    endpoint2 = foot_point + d2 * line_dir

    wall_endpoints = np.array([endpoint1, endpoint2])
    return wall_endpoints

def is_same_point(point_a, point_b, eps_dist = 100):
    """Determine if two points are the same based on distance criteria. Unit: cm"""
    if len(point_a) == 0 or len(point_b) == 0:
        return np.bool_(False)
    # Convert to numpy arrays before calculation
    point_a_np = np.array(point_a)
    point_b_np = np.array(point_b)
    dist = np.linalg.norm(point_a_np - point_b_np)
    if dist < eps_dist:
        return np.bool_(True)
    return np.bool_(False)

def merge_wall_points(all_points):
    [vx, vy, x0, y0] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)
    line_dir = np.array([vx[0], vy[0]])
    line_point = np.array([x0[0], y0[0]])
    projections = np.dot(all_points - line_point, line_dir)
    min_proj = np.min(projections)
    max_proj = np.max(projections)
    endpoint1 = line_point + min_proj * line_dir
    endpoint2 = line_point + max_proj * line_dir
    endpoints = np.array([endpoint1, endpoint2])
    return endpoints
