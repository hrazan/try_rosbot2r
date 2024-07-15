import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'try_rosbot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'components'), glob('urdf/components/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='husarion',
    maintainer_email='husarion@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_control_node = try_rosbot_pkg.lidar_control_node:main",
            "range_control_node = try_rosbot_pkg.range_control_node:main",
            "camera_control_node = try_rosbot_pkg.camera_control_node:main",
            "movement_control_node = try_rosbot_pkg.movement_control_node:main",
            "tracker_node = try_rosbot_pkg.tracker_node:main",
        ],
    },
)
