from setuptools import setup

package_name = 'astora_lights'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karoline Heiwolt',
    maintainer_email='karoline@heiwolt.de',
    description='ROS2 package for interfacing one or multiple ASTORA soft panels via DMX',
    license='MIT License, Copyright 2023 Karoline Heiwolt',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astora_node = astora_lights.astora_node:main'
        ],
    },
)