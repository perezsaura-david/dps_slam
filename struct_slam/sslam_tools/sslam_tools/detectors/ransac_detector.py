# Copyright 2026 Universidad Politecnica de Madrid (UPM).
#
# Author: Pedro Espinosa Angulo
# Contributor: Guanliang Li, Santiago Tapia Fernandez (supervised)
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
import cv2
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.detector import Detector, DetectorConfig, SSCollection
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.imagebased_detector import ImageBasedDetector
from dataclasses import dataclass

@dataclass
class RANSACDetectorConfig(DetectorConfig):
    # Debug image parameters
    virtual_canvas_size: int = 4096
    sensor_posx: int = 2048
    sensor_posy: int = 2048
    # Detector config
    DBSCAN_eps: float = 30.0 # cm
    DBSCAN_min_samples: int = 5
    RANSAC_residual_threshold: float = 15.0 # cm
    RANSAC_min_samples: int = 2
    length_threshold: float = 70.0 # cm # to distinguish walls and other objects

class RANSACDetector(Detector[RANSACDetectorConfig]):
    def __init__(self, name: str, config: RANSACDetectorConfig):
        super().__init__(name, config)

        self.dbscan = DBSCAN(eps=self.config.DBSCAN_eps, 
                             min_samples=self.config.DBSCAN_min_samples)
        
        self.xdata = np.array([], dtype=np.float32)
        self.ydata = np.array([], dtype=np.float32)
        self.ransac_result_ssc: SSCollection = SSCollection()
        
    @classmethod
    def get_config_class(cls) -> type[DetectorConfig]:
        return RANSACDetectorConfig
    
    def _do_detect(self, xdata: np.ndarray, ydata: np.ndarray) -> SSCollection:
        points_np = np.vstack((xdata, ydata)).T  # Shape (N, 2)
        return self.inference(points_np)
    
    def inference(self, points_np: np.ndarray) -> SSCollection:
        # Check points_np shape
        if points_np.shape[1] != 2:
            raise ValueError(f"points_np must have shape (N, 2). Got: {points_np.shape}")
        
        # DBScan Clustering
        if len(points_np) > 0:
            labels = self.dbscan.fit_predict(points_np)
        else:
            return SSCollection()  # Return empty collection if no points
        
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # Remove noise label
        
        ss_collection = SSCollection()
        for label in unique_labels:
            mask = (labels == label)
            cluster_points = points_np[mask]

            if len(cluster_points) < 3:
                continue  # Need at least 3 points to fit a line

            X = cluster_points[:, 0:1]
            y = cluster_points[:, 1]

            ransac = RANSACRegressor(residual_threshold=self.config.RANSAC_residual_threshold, 
                                     min_samples=self.config.RANSAC_min_samples)
            ransac.fit(X, y)

            inlier_mask = ransac.inlier_mask_
            inliers = cluster_points[inlier_mask]

            if len(inliers) < 2:
                continue  # Not enough inliers to define a line

            slope = ransac.estimator_.coef_[0]

            v = np.array([1, slope])
            norm_v = v / np.linalg.norm(v)

            origin = inliers[0]
            vecs = inliers - origin
            projections = np.dot(vecs, norm_v)
            
            min_idx = np.argmin(projections)
            max_idx = np.argmax(projections)

            p1 = inliers[min_idx]
            p2 = inliers[max_idx]
            
            length_cm = np.linalg.norm(p2 - p1)

            if length_cm > self.config.length_threshold:
                ss_collection.endpoints.extend([float(p1[0]), float(p1[1]), float(p2[0]), float(p2[1])])
            else:
                center_x, center_y = np.mean(inliers, axis=0)
                center_x, center_y = float(center_x), float(center_y)
                ss_collection.others_circles_centers.extend([center_x, center_y])
                ss_collection.others_circles_radius.append(float(length_cm / 2))

        self.xdata = points_np[:, 0]
        self.ydata = points_np[:, 1]
        self.ransac_result_ssc = ss_collection
        
        return ss_collection
        
    def _build_debug_image(self) -> np.ndarray:
        # Build debug image showing points
        results_image: np.ndarray = ImageBasedDetector.build_image(self.xdata, self.ydata,
                                                                   self.config.virtual_canvas_size,
                                                                   self.config.result_img_size,
                                                                   self.config.sensor_posx,
                                                                   self.config.sensor_posy,
                                                                   self.config.struct_radius)

        results_image = cv2.cvtColor(results_image, cv2.COLOR_GRAY2BGR)
        # Draw detected endpoints
        for i in range(0, len(self.ransac_result_ssc.endpoints), 4):
            x1, y1, x2, y2 = self.ransac_result_ssc.endpoints[i:i+4]
            p1_img = self.laser_to_img(x1, y1)
            p2_img = self.laser_to_img(x2, y2)
            cv2.line(results_image, (int(p1_img[0]), int(p1_img[1])), 
                     (int(p2_img[0]), int(p2_img[1])), (255, 0, 0), 2)

        # Draw detected circles for others
        for i in range(0, len(self.ransac_result_ssc.others_circles_centers), 2):
            center_x = self.ransac_result_ssc.others_circles_centers[i]
            center_y = self.ransac_result_ssc.others_circles_centers[i+1]
            radius = self.ransac_result_ssc.others_circles_radius[i // 2]
            center_img = self.laser_to_img(center_x, center_y)
            scale_ratio = self.config.result_img_size / self.config.virtual_canvas_size
            cv2.circle(results_image, (int(center_img[0]), int(center_img[1])), 
                       int(radius * scale_ratio), (0, 255, 0), 2)

        return results_image

    def laser_to_img(self, xdata: float, ydata: float) -> np.array:
        virtual_canvas_size = self.config.virtual_canvas_size
        result_img_size = self.config.result_img_size
        sensor_posx = self.config.sensor_posx
        sensor_posy = self.config.sensor_posy

        ratio = result_img_size / virtual_canvas_size

        raw_x = int((sensor_posx + xdata) * ratio)
        raw_y = int((sensor_posy - ydata) * ratio)

        flag_x = (raw_x >= 0) and (raw_x < result_img_size)
        flag_y = (raw_y >= 0) and (raw_y < result_img_size)

        if not flag_x:
            raw_x = -1
        if not flag_y:
            raw_y = -1
        points_img = np.array([raw_x, raw_y])

        return points_img
