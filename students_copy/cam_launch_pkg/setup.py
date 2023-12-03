from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'cam_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #This finds the .py launch files in the launch directory
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), #This finds the .yaml config files in the config directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shalman_khan',
    maintainer_email='shabashkhan@artc.a-star.edu.sg',
    description='Package to launch usb cam driver and to show cam image',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
