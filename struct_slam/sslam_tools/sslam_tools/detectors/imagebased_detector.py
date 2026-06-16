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

from abc import abstractmethod
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.detector import Detector, DetectorConfig, SSCollection
from dataclasses import dataclass, field
from typing import TypeVar, Generic


@dataclass
class ImageBasedDetectorConfig(DetectorConfig):
    base_img_size: int = 4096
    processed_img_size: int = 640
    sensor_posx: int = 2048
    sensor_posy: int = 2048

    kernel_size: int = 3 
    iterations: int = 1
    smoothing: str = 'none' #'box'  # Options: 'none', 'gaussian', 'median', 'bilateral', 'box'
    sigma: float = 1.0
    ksize_smoothing: int = 1
    last_dilation: str = 'none'  # Options: 'none', 'gaussian', 'median', 'bilateral', 'box'
    truncate: bool = True
    thresh: int = 210

U = TypeVar('U', bound=ImageBasedDetectorConfig)

class ImageBasedDetector(Detector, Generic[U]):
    def __init__(self, name: str, config: dict):
        super().__init__(name, config)
        self.config: U = self.config
        self.ratio_bp: float = self.config.base_img_size / self.config.processed_img_size
        self.ratio_pr: float = self.config.result_img_size / self.config.processed_img_size

        self.lidar_image_buffer: np.ndarray = np.full((self.config.base_img_size, self.config.base_img_size), 255, dtype=np.uint8)
        self.raw_image: np.ndarray = np.full((self.config.processed_img_size, self.config.processed_img_size), 255, dtype=np.uint8)
        self.processed_image: np.ndarray = np.full((self.config.processed_img_size, self.config.processed_img_size), 255, dtype=np.uint8)

        self.tmp_rimage: np.ndarray = np.empty((self.config.result_img_size, self.config.result_img_size), dtype=np.uint8)
        self.use_tmp_rimage: bool = self.processed_image.shape != self.tmp_rimage.shape

    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        return ImageBasedDetector.image_noise_processing(
            image,
            kernel_size=self.config.kernel_size,
            iterations=self.config.iterations,
            smoothing=self.config.smoothing,
            sigma=self.config.sigma,
            ksize_smoothing=self.config.ksize_smoothing,
            last_dilation=self.config.last_dilation,
            truncate=self.config.truncate,
            thresh=self.config.thresh
        )

    @classmethod
    def get_config_class(cls) -> type[DetectorConfig]:
        return ImageBasedDetectorConfig

    def _do_detect(self, xdata: np.ndarray, ydata: np.ndarray) -> SSCollection:
        self.raw_image = ImageBasedDetector.build_image(xdata, ydata, 
                                                        self.config.base_img_size,
                                                        self.config.processed_img_size,
                                                        self.config.sensor_posx,
                                                        self.config.sensor_posy,
                                                        self.config.struct_radius)
        np.copyto(self.processed_image, self.raw_image) # copy raw image before preprocessing (for debugging)
        self.preprocess_image(self.processed_image)
        return self.inference(self.processed_image)

    def img_to_laser(self, vertices_img):
        # Scale back to base image resolution before shifting to sensor center
        x_base = vertices_img[:, 0] * self.ratio_bp
        y_base = vertices_img[:, 1] * self.ratio_bp

        x_laser = x_base - self.config.sensor_posx
        y_laser = self.config.sensor_posy - y_base

        return np.column_stack((x_laser, y_laser)) 

    @abstractmethod
    def inference(self, processed_image: np.ndarray) -> SSCollection:
        pass

    # ===================================== #
    # =  AUX METHODS FOR IMAGE DENOISING  = #
    # ===================================== #

    @staticmethod
    def build_image(xdata: np.ndarray, ydata: np.ndarray, 
                    img_size: int = 4096, processed_img_size: int = 640,
                    sensor_posx: int = 2048, sensor_posy: int = 2048, 
                    struct_radius: int = 1) -> np.ndarray:
        erode_kernel = np.array([[0, 1, 0],
                                 [1, 1, 1],
                                 [0, 1, 0]], dtype=np.uint8)
        # convert to pixel coordinates
        raw_x = ((sensor_posx + xdata) * processed_img_size / img_size).astype(np.int32)
        raw_y = ((sensor_posy - ydata) * processed_img_size / img_size).astype(np.int32)

        # filter out-of-bounds points
        mask = (raw_x >= 0) & (raw_x < processed_img_size) & (raw_y >= 0) & (raw_y < processed_img_size)
        pix_x: np.ndarray = raw_x[mask]
        pix_y: np.ndarray = raw_y[mask]

        raw_image: np.ndarray = np.full((processed_img_size, processed_img_size), 255, dtype=np.uint8)
        raw_image[pix_y, pix_x] = 0
        cv2.erode(raw_image, erode_kernel, iterations=struct_radius, dst=raw_image)
        return raw_image

    @staticmethod
    def apply_smoothing(image: np.ndarray, method: str='none', ksize: int=3, sigma: float=1.0) -> np.ndarray:
        """
        Apply smoothing using OpenCV.
        method: 'none', 'gaussian', 'median', 'bilateral', 'box'
        ksize: kernel size (int). For Gaussian/box/median: odd integer. For bilateral, interpreted as d.
        sigma: sigma for Gaussian (sigmaX). Ignored by median/box.
        """
        if method == 'none' or ksize is None or ksize <= 0:
            return image
        if method == 'gaussian':
            k = (ksize, ksize)
            return cv2.GaussianBlur(image, k, sigmaX=sigma, dst=image)
        if method == 'median':
            # medianBlur requires odd ksize
            k = ksize if ksize % 2 == 1 else ksize + 1
            return cv2.medianBlur(image, k, dst=image)
        if method == 'bilateral':
            return cv2.bilateralFilter(image, d=ksize, sigmaColor=sigma*10, sigmaSpace=sigma*10, dst=image)
        if method == 'box':
            return cv2.blur(image, (ksize, ksize), dst=image)
        return image

    @staticmethod
    def image_noise_processing(image: np.ndarray, kernel_size: int, iterations: int,
                               smoothing: str, sigma: float, ksize_smoothing: int,
                               last_dilation: str, truncate: bool, thresh: int) -> np.ndarray:
        # 1. Define the kernel (structuring element)
        # A rectangular kernel of ones is standard for max pooling logic
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        # 2. Apply smoothing (optional)
        image = ImageBasedDetector.apply_smoothing(image, method=smoothing, ksize=ksize_smoothing, sigma=sigma)

        # 3. Truncate image
        if truncate:
            _, image = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY, dst=image)

        # 4. Apply Erosion
        # Erosion takes the minimum value in the neighborhood defined by the kernel
        image = cv2.erode(image, kernel, iterations=iterations, dst=image)

        # 5. Pass filter again to smooth edges (optional)
        image = ImageBasedDetector.apply_smoothing(image, method=last_dilation, ksize=ksize_smoothing, sigma=sigma)
        return image
  