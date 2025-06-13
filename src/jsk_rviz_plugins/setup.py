from setuptools import setup

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
        ('share/' + package_name + '/launch', ['launch/*.launch.py']),
        ('share/' + package_name + '/config', ['config/*']),
        ('share/' + package_name + '/samples', ['samples/*']),
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
            'overlay_sample = jsk_rviz_plugins.samples.overlay_sample:main',
            'piechart_sample = jsk_rviz_plugins.samples.piechart_sample:main',
            'static_overlay_text = jsk_rviz_plugins.scripts.static_overlay_text:main',
            'rosconsole_overlay_text = jsk_rviz_plugins.scripts.rosconsole_overlay_text:main',
        ],
    },
)
