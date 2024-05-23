from setuptools import find_packages, setup

package_name = 'lab6_nodes'

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
    maintainer='tp03',
    maintainer_email='zalewski.tomek03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'marker_broker = lab6_nodes.marker_broker:main',
                'final_boss = lab6_nodes.final_boss:main',
        ],
    },
)
