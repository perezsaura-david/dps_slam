import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sslam_tools'

def list_data_files(source_dir, target_dir, excluded_files=[]):
    """Recursively walk through a directory and generate a list of tuples (destination, [files])"""
    data_files = []
    for root, _, files in os.walk(source_dir):
        if files:
            rel_dir = os.path.relpath(root, source_dir) # relative path from source_dir
            install_path = os.path.join(target_dir, rel_dir) # target installation path inside share/<package_name>/config/
            file_list = [os.path.join(root, f) for f in files if f not in excluded_files] # full paths to files
            data_files.append((install_path, file_list))

    return data_files

resource_files = list_data_files(
    source_dir=os.path.join('resource'),
    target_dir=os.path.join('share', package_name, 'resource'),
    excluded_files=[package_name]
)

config_files = list_data_files(
    source_dir=os.path.join('config'),
    target_dir=os.path.join('share', package_name, 'config')
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        *resource_files,
        *config_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guanliang Li, Pedro Espinosa Angulo, Santiago Tapia Fernández',
    maintainer_email='guanliang.li@alumnos.upm.es, pedro.espinosa@upm.es, stapia@fi.upm.es',
    description='This package converts raw LiDAR range information r into a 2D bird\'s-eye view (BEV) map.',
    license='GPL-3.0-or-later',
    extras_require={"test": ["pytest"]},
    entry_points={
        'console_scripts': [
            'detector_node = sslam_tools.detector_node:main',
            'local_fusion = sslam_tools.local_fusion:main',
            'global_fusion = sslam_tools.global_fusion:main',
            'eval_node = sslam_tools.eval:main',
            'image_saver = sslam_tools.image_saver:main',

            'detector_node_3D_RANSAC = sslam_tools.detector_node_3D_RANSAC:main',
            'detector_node_3D_RegionGrow = sslam_tools.detector_node_3D_RegionGrow:main',
        ],
    },
)
