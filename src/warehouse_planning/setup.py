from setuptools import find_packages, setup

package_name = 'warehouse_planning'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/warehouse_graph.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Péter Varga',
    maintainer_email='vargapeter389@gmail.com',
    description='Validated warehouse aisle graph and shortest-path planning.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'route_example = warehouse_planning.route_example:main',
    ]},
)
