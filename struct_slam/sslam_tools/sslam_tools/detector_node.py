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


import rclpy
from rclpy.node import Node
from msg_interfaces.msg import LidarData, Objects
from sensor_msgs.msg import Image

import numpy as np
import os, cv2, time, psutil, threading, importlib
from types import MappingProxyType

from dataclasses import fields
from workspace.src.dps_slam.struct_slam.sslam_tools.sslam_tools.detectors.detector import SSCollection, Detector

class Detectors_Node(Node):
    detectors_registry = MappingProxyType({
        'yolo': ('sslam_tools.detectors.yolo_detector', 'YoloDetector'),
        'lsd': ('sslam_tools.detectors.lsd_detector', 'LSDDetector'),
        'hought': ('sslam_tools.detectors.houghT_detector', 'HoughTDetector'),
        'ransac': ('sslam_tools.detectors.ransac_detector', 'RANSACDetector'),
    })

    def __init__(self):
        super().__init__('detector_node')
        self.subscription = self.create_subscription(
            LidarData,
            'lidar_data',
            self.listener_callback,
            10
        )

        self.declare_parameter('detector_type', 'yolo')
        detector_type: str = self.get_parameter('detector_type').value.lower()
        if detector_type not in self.detectors_registry:
            self.get_logger().error(f'Unsupported detector type: {detector_type}')
            raise ValueError(f'Unsupported detector type: {detector_type}')

        module_path, class_name = self.detectors_registry[detector_type]
        try:
            module = importlib.import_module(module_path)
            detector_cls: Detector = getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f'Failed to load detector {detector_type} from {module_path}: {e}')
            raise

        config_dict: dict = {}
        config_str: str = 'DETECTOR CONFIGURATION:\n'
        for field in fields(detector_cls.get_config_class()):
            self.declare_parameter(field.name, field.default)
            config_dict[field.name] = self.get_parameter(field.name).value
            config_str += f'\t{field.name}: {config_dict[field.name]}\n'

        self.detector: Detector = detector_cls(f'{detector_type}_detector', config_dict)
        self.process = psutil.Process(os.getpid())
        self.debug_image = np.empty(self.detector.get_debug_image_shape(), dtype=np.uint8)
        self.worker_image = self.debug_image.copy()

        self.image_publisher = self.create_publisher(Image, 'lidar_image', 10)
        self.detector_publisher = self.create_publisher(Objects, 'detection_results', 10)

        self.image_lock = threading.Lock()
        self.new_image_event = threading.Event()
        self.image_publish_thread = threading.Thread(target=self.image_worker_loop, daemon=True) 

        self.get_logger().info(f'Using {detector_type.upper()} as Detector Node.')
        self.get_logger().info(config_str)
        if self.detector.config.capture_inference_image:
            self.image_publish_thread.start()
            self.get_logger().info('Image publishing thread started for debug images.')

    def listener_callback(self, msg):
        detection_results = Objects()
        detection_results.header = msg.header
        detection_results.method = self.detector.name 
        
        start_time = time.time()

        x_data = np.array(msg.x_data, dtype=np.float32)
        y_data = np.array(msg.y_data, dtype=np.float32)

        # transform mm to cm
        x = x_data / 10
        y = y_data / 10

        sscollection: SSCollection = self.detector.detect(x, y)
        if self.detector.config.capture_inference_image:
            with self.image_lock:
                self.debug_image = self.detector.get_debug_image(self.debug_image)
            self.new_image_event.set()

        detection_results.endpoints = sscollection.endpoints
        detection_results.columns_circles_centers = sscollection.columns_circles_centers
        detection_results.columns_circles_radius = sscollection.columns_circles_radius
        detection_results.others_circles_centers = sscollection.others_circles_centers
        detection_results.others_circles_radius = sscollection.others_circles_radius

        end_time = time.time()
        self.detector_publisher.publish(detection_results)

        # cpu_usage = self.process.cpu_percent(interval=None)
        # mem_usage = self.process.memory_percent()

        self.get_logger().info(f'{self.detector.name.upper()} detection elapsed time: {end_time - start_time:.4f} seconds')
        # self.get_logger().info(f'{self.detector.name.upper()} detection elapsed time: {end_time - start_time:.4f} seconds, CPU usage: {cpu_usage:.2f}%, Memory usage: {mem_usage:.2f}%')

    def image_worker_loop(self):
        while rclpy.ok():
            if self.new_image_event.wait(timeout=1.0):
                self.new_image_event.clear()
                with self.image_lock:
                    np.copyto(self.worker_image, self.debug_image)

                self.publish_ros_image(self.worker_image, encoding='bgr8')

    def publish_ros_image(self, image_np, encoding='bgr8'):
        image_np = cv2.resize(image_np, (640, 640))

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        
        msg.height = image_np.shape[0]
        msg.width = image_np.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        
        if encoding == 'mono8':
            msg.step = msg.width
        elif encoding == 'bgr8' or encoding == 'rgb8':
            msg.step = msg.width * 3
            
        msg.data = image_np.tobytes()
        self.image_publisher.publish(msg)
  
def main(args=None):
    rclpy.init(args=args)
    detectors_node = Detectors_Node()
    rclpy.spin(detectors_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
