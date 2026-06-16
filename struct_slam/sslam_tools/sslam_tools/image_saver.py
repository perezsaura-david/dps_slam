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
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge # Avoid using cv_bridge due to numpy version mismatch
import cv2
import sys
import numpy as np

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # Parametros para configurar el topico y el nombre de archivo
        self.declare_parameter('topic_name', '/lidar_image')
        self.declare_parameter('output_file', 'captured_image.png')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        # self.bridge = CvBridge()
        self.get_logger().info(f'Esperando imagen en el topic: {topic_name}')
        self.get_logger().info(f'La imagen se guardara en: {self.output_file}')

    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV MANUALMENTE (sin cv_bridge)
            # Asumimos uint8
            dtype = np.uint8
            n_channels = 1
            if msg.encoding == 'bgr8':
                n_channels = 3
            elif msg.encoding == 'rgb8':
                n_channels = 3
            elif msg.encoding == 'mono8':
                n_channels = 1
            else:
                 # Default if unknown, assuming 3 channels for visualization
                 # O podemos intentar inferir de step/width
                 if msg.step == msg.width * 3:
                     n_channels = 3
                 else:
                     n_channels = 1
            
            # Reconstruir array de numpy
            # msg.data es array directo de bytes
            image_data = np.frombuffer(msg.data, dtype=dtype)
            
            if n_channels > 1:
                cv_image = image_data.reshape((msg.height, msg.width, n_channels))
            else:
                cv_image = image_data.reshape((msg.height, msg.width))

            # Si es RGB, convertir a BGR para guardar con OpenCV
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            
            # Guardar la imagen
            cv2.imwrite(self.output_file, cv_image)
            self.get_logger().info(f'Imagen guardada exitosamente en {self.output_file}')
            
            # Terminar el nodo despues de guardar
            # Levantamos SystemExit para salir limpiamente del spin
            raise SystemExit 
            
        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f'Error al guardar la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    
    try:
        rclpy.spin(image_saver)
    except SystemExit:
        rclpy.logging.get_logger("image_saver").info('Cerrando nodo...')
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
