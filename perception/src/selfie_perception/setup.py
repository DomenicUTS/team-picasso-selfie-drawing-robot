from setuptools import setup
from glob import glob
import os

package_name = 'selfie_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rembg[cpu]',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description=(
        'Selfie perception pipeline — background removal (rembg), '
        'Canny edge detection (sigma=3), stroke extraction for UR3 drawing robot'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_loader_node = selfie_perception.image_loader_node:main',
            'perception_node = selfie_perception.perception_node:main',
            'visualization_node = selfie_perception.visualization_node:main',
            'pipeline = selfie_perception.pipeline:main',
        ],
    },
)
