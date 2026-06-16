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

from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.detector import SSCollection
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.imagebased_detector import ImageBasedDetector, ImageBasedDetectorConfig
from dataclasses import dataclass

@dataclass
class LSDDetectorConfig(ImageBasedDetectorConfig):
    pass

class LSDDetector(ImageBasedDetector[LSDDetectorConfig]):
    def __init__(self, name: str, config: dict):
        super().__init__(name, config)
        self.lsd: cv2.LineSegmentDetector = cv2.createLineSegmentDetector(refine=cv2.LSD_REFINE_STD)
        self.lsd_result_lines: list = []

    @classmethod
    def get_config_class(cls) -> type[ImageBasedDetectorConfig]:
        return LSDDetectorConfig

    def inference(self, processed_image: np.ndarray) -> SSCollection:
        lines, _, _, _ = self.lsd.detect(processed_image)

        ss_collection = SSCollection()
        if lines is not None:
            for line in lines:
                pts = line[0].reshape(2, 2)
                laser_pts = self.img_to_laser(pts)
                ss_collection.endpoints.extend(laser_pts.ravel().astype(float).tolist())

        self.lsd_result_lines = lines
        return ss_collection
    
    def _build_debug_image(self) -> np.ndarray:
        image_to_use = self.processed_image
        if self.use_tmp_rimage:
            cv2.resize(self.processed_image, 
                       (self.config.result_img_size, self.config.result_img_size), 
                       dst=self.tmp_rimage)
            image_to_use = self.tmp_rimage

        cv2.cvtColor(image_to_use, cv2.COLOR_GRAY2BGR, dst=self.results_image)
        if self.lsd_result_lines is not None:
            for line in self.lsd_result_lines:
                x1, y1, x2, y2 = line[0]
                # Scale coordinates to the results image size
                cv2.line(self.results_image, 
                         (int(x1 * self.ratio_pr), int(y1 * self.ratio_pr)), 
                         (int(x2 * self.ratio_pr), int(y2 * self.ratio_pr)), 
                         (0, 255, 0), 2)
        return self.results_image