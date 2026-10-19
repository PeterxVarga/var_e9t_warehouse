from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Péter Varga',
    maintainer_email='vargapeter389@gmail.com',
    description='Odometry-based goal and aisle-route tracking for the warehouse robot.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'heading_odometry = warehouse_control.heading_odometry_node:main',
        'goal_controller = warehouse_control.goal_controller:main',
        'route_controller = warehouse_control.route_controller:main',
        'mission_controller = warehouse_control.mission_controller:main',
    ]},
)
