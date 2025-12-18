from setuptools import setup
import os
from glob import glob

package_name = 'livox_carto'

setup(
   name=package_name,
   version='0.0.0',
   packages=[],
   data_files=[
       ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
       (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
   ],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='aitec',
   maintainer_email='aitec@todo.todo',
   description='Livox LiDAR + Cartographer 2D SLAM launch files',
   license='MIT',
   tests_require=['pytest'],
   entry_points={},
)

 

