from setuptools import find_packages, setup

package_name = 'warehouse_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Péter Varga',
    maintainer_email='vargapeter389@gmail.com',
    description='Odometry-based goal tracking for the warehouse robot.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'goal_controller = warehouse_control.goal_controller:main',
    ]},
)
