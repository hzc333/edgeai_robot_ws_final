from glob import glob

from setuptools import find_packages, setup

package_name = 'edgeai_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hzc333',
    maintainer_email='huangzc333@gmail.com',
    description='Robot bringup, safety watchdog and robot agent',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_watchdog = edgeai_robot_bringup.safety_watchdog:main',
            'robot_agent = edgeai_robot_bringup.robot_agent:main',
        ],
    },
)
