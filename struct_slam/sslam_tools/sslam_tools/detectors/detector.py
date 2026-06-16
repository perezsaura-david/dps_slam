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
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, fields
from typing import TypeVar, Generic

@dataclass
class DetectorConfig:
    capture_inference_image: bool = True
    result_img_size: int = 640
    struct_radius: int = 1

@dataclass
class SSCollection:
    endpoints: list = field(default_factory=list)
    columns_circles_centers: list = field(default_factory=list)
    columns_circles_radius: list = field(default_factory=list)
    others_circles_centers: list = field(default_factory=list)
    others_circles_radius: list = field(default_factory=list)

T = TypeVar('T', bound=DetectorConfig)

class Detector(ABC, Generic[T]):
    def __init__(self, name: str, config: dict):
        super().__init__()
        self.name: str = name
        self.config: T = self._load_config(config)
        self.results_image: np.ndarray = np.full((self.config.result_img_size, 
                                                  self.config.result_img_size, 3), 
                                                  255, dtype=np.uint8)

    def _load_config(self, config_dict: dict) -> T:
        config_cls = self.get_config_class()
        valid_keys = {f.name for f in fields(config_cls)}
        filtered_config = {k: v for k, v in config_dict.items() if k in valid_keys}
        
        return config_cls(**filtered_config)

    def detect(self, xdata: np.ndarray, ydata: np.ndarray) -> SSCollection:
        if not isinstance(xdata, np.ndarray) or not isinstance(ydata, np.ndarray):
            raise TypeError(f"x & y data MUST be numpy arrays. Got: x={type(xdata)} y={type(ydata)}")
        
        if xdata.ndim != 1 or ydata.ndim != 1:
            raise ValueError(f"Arrays must be 1D. Got dimensions: x={xdata.ndim}, y={ydata.ndim}")
             
        if xdata.size != ydata.size:
            raise ValueError(f"x and y must have same size. Got: x={xdata.size}, y={ydata.size}")

        ssc = self._do_detect(xdata.astype(np.float32), ydata.astype(np.float32))

        if self.config.capture_inference_image:
            self.results_image = self._build_debug_image()
        return ssc

    @classmethod
    @abstractmethod
    def get_config_class(cls) -> type[T]:
        pass

    @abstractmethod
    def _do_detect(self, xdata: np.ndarray, ydata: np.ndarray) -> SSCollection:
        pass

    @abstractmethod
    def _build_debug_image(self, ) -> np.ndarray:
        pass

    def get_debug_image(self, buffer_to_store_img: np.ndarray) -> np.ndarray:
        if buffer_to_store_img.shape != self.results_image.shape:
            raise ValueError(f"Buffer shape mismatch. Expected: {self.results_image.shape}, Got: {buffer_to_store_img.shape}")

        np.copyto(buffer_to_store_img, self.results_image)
        return buffer_to_store_img

    def get_debug_image_shape(self) -> tuple[int, int, int]:
        return self.results_image.shape