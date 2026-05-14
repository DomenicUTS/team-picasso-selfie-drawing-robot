from setuptools import setup, find_packages

package_name = 'ur3_motion_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/ur3_motion_planning_moveit2.launch.py',
             'launch/integrated_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Domenic Kadioglu',
    maintainer_email='domenic@utsrobo.com',
    description='UR3 Motion Planning and Control for Team Picasso',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_node = ur3_motion_planning.ur3_drawing_node:main',
            'scene_publisher = ur3_motion_planning.scene_publisher:main',
        ],
    },
)
