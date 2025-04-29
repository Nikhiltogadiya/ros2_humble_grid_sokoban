from setuptools import find_packages, setup
import os
from glob import glob
from camera_pkg.sokoban.sk_game import SokobanGame

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # *** Installs ALL '.launch.py' files from your NEW 'launch' folder ***
        # It puts them into 'install/camera_pkg/share/camera_pkg/launch/'
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # *** Installs ALL '.yaml' files from your NEW 'config' folder ***
        # It puts them into 'install/camera_pkg/share/camera_pkg/config/'
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # (os.path.join('share', package_name, 'maps'), glob('maps/*.csv')),
    ],
    install_requires=['setuptools',
                      'opencv-python',],
    zip_safe=True,
    maintainer='niks_autonomous',
    maintainer_email='niks_autonomous@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = camera_pkg.camera_subscriber:main',
            # 'grid_detection = camera_pkg.grid_detection_2:main',
            'edge_detection = camera_pkg.edge_detection:main',
            # 'lane_publisher = camera_pkg.lane_publisher:main',
            'lane_centering_node = camera_pkg.lane_centering_node:main',
            'grid_navigator = camera_pkg.grid_navigator:main', # for exercise 1
            'turn_controller = camera_pkg.turn_controller:main',
            'grid_navigator_A2 = camera_pkg.grid_navigator_A2:main',
            'sokoban_navigator = camera_pkg.sokoban_navigator:main'
        ],
    },
)
