from setuptools import find_packages, setup

package_name = 'edgeai_rescue_coordinator'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['schemas/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'jsonschema>=4.18,<5'],
    extras_require={'test': ['pytest<9']},
    zip_safe=False,
    maintainer='EdgeAI team',
    maintainer_email='maintainers@example.invalid',
    description='Deterministic pure Python mission plan validator',
    license='Apache-2.0',
)
