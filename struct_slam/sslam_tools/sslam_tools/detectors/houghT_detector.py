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


import cv2
import numpy as np

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.detector import SSCollection
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.imagebased_detector import ImageBasedDetector, ImageBasedDetectorConfig
from dataclasses import dataclass

@dataclass
class HoughTDetectorConfig(ImageBasedDetectorConfig):
    rho: float = 1.0
    theta: float = np.pi / 180
    threshold: int = 50
    min_line_length: float = 10
    max_line_gap: float = 20

class HoughTDetector(ImageBasedDetector[HoughTDetectorConfig]):
    def __init__(self, name: str, config: dict):
        super().__init__(name, config)
        self.edges_buffer = np.zeros((self.config.processed_img_size, self.config.processed_img_size), dtype=np.uint8)
        self.hought_result_lines: list = []

    @classmethod
    def get_config_class(cls) -> type[ImageBasedDetectorConfig]:
        return HoughTDetectorConfig

    def inference(self, processed_image: np.ndarray) -> SSCollection:
        cv2.Canny(processed_image, 50, 200, edges=self.edges_buffer)

        lines = cv2.HoughLinesP(
            self.edges_buffer,
            rho= self.config.rho,
            theta= self.config.theta,
            threshold= self.config.threshold,
            minLineLength= self.config.min_line_length,
            maxLineGap= self.config.max_line_gap
        )

        ss_collection = SSCollection()
        if lines is not None:
            for line in lines:
                pts = line[0].reshape(2, 2)
                laser_pts = self.img_to_laser(pts)
                ss_collection.endpoints.extend(laser_pts.ravel().astype(float).tolist())

        self.hought_result_lines = lines
        return ss_collection
    
    def _build_debug_image(self) -> np.ndarray:
        image_to_use = self.processed_image
        if self.use_tmp_rimage:
            cv2.resize(self.processed_image,
                       (self.config.result_img_size, self.config.result_img_size),
                       dst=self.tmp_rimage
                       )
            image_to_use = self.tmp_rimage

        cv2.cvtColor(image_to_use, cv2.COLOR_GRAY2BGR, dst=self.results_image)
        if self.hought_result_lines is not None:
            for line in self.hought_result_lines:
                x1, y1, x2, y2 = line[0]
                # Scale coordinates to the results image size
                cv2.line(self.results_image, 
                         (int(x1 * self.ratio_pr), int(y1 * self.ratio_pr)), 
                         (int(x2 * self.ratio_pr), int(y2 * self.ratio_pr)), 
                         (0, 0, 255), 2)
        return self.results_image

