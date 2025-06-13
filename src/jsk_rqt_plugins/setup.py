from setuptools import setup

package_name = 'jsk_rqt_plugins'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryohei Ueda',
    maintainer_email='ueda@jsk.t.u-tokyo.ac.jp',
    description='The jsk_rqt_plugins package (ROS2 port)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
