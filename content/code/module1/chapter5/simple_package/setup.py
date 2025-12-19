from setuptools import setup

package_name = 'simple_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_launch.py']),
        ('share/' + package_name + '/launch', ['launch/simple_launch.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User Name',
    maintainer_email='user@example.com',
    description='Simple package example for Chapter 5: ROS 2 Packages and Launch Files',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_listener = simple_package.simple_listener:main',
        ],
    },
)