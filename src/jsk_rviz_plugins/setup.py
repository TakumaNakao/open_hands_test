from setuptools import setup
import os
from glob import glob

package_name = 'jsk_rviz_plugins'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    package_dir={'': 'python'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch_py/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/samples', glob('samples/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
        ('share/' + package_name + '/icons', glob('icons/*')),
        ('share/' + package_name + '/resources', glob('resources/**/*', recursive=True)),
        ('lib/' + package_name, glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kei Okada',
    maintainer_email='k-okada@jsk.t.u-tokyo.ac.jp',
    description='The jsk_rviz_plugins package (ROS2 port)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Note: Entry points would need main() functions in the respective files
            # For now, scripts are installed directly via CMakeLists.txt
        ],
    },
)
