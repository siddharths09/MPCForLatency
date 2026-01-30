from setuptools import find_packages, setup

package_name = 'my_turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarveshv',
    maintainer_email='sarveshv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = my_turtle_controller.turtle_controller:main',
            'network_delay_node = my_turtle_controller.network_delay_node:main',
            'jitter_node = my_turtle_controller.jitter_node:main',
            'tunnel_node = my_turtle_controller.tunnel_node:main',
            'mpc_supervisor_node = my_turtle_controller.mpc_supervisor_node:main',
            'obstacle_visualizer = my_turtle_controller.obstacle_visualizer:main',
        ],
    },
)
