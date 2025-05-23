from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'vision_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_launch.py','launch/rviz_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/my_custom_world.world']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yousuf',
    maintainer_email='yousaf4it@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Line_follower = vision_line_follower.Line_follower:main'
        ],
    },
)
