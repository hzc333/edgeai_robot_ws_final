# Copyright 2026 EdgeAI Robotics Challenge Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob
import shlex
import subprocess

from setuptools import Extension, find_packages, setup

package_name = 'edgeai_task_coordinator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[Extension(
        'edgeai_task_coordinator._gazebo_contacts',
        ['gazebo_plugin/Contacts.cc'],
        language='c++',
        extra_compile_args=shlex.split(subprocess.check_output(
            ['pkg-config', '--cflags', 'gz-sim8', 'gz-plugin2'], text=True)) + ['-std=c++17'],
        extra_link_args=shlex.split(subprocess.check_output(
            ['pkg-config', '--libs', 'gz-sim8', 'gz-plugin2'], text=True)),
    )],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/test_assets', glob('test_assets/*')),
        ('share/' + package_name + '/gazebo_plugin', glob('gazebo_plugin/*.cc')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EdgeAI Team',
    maintainer_email='edgeai@example.invalid',
    description='Task allocation, rescue execution and verification',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'coordinator = edgeai_task_coordinator.coordinator:main',
        'observer_plugin = edgeai_task_coordinator.physics:main',
    ]},
)
