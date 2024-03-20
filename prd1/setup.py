import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'prd1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tp03',
    maintainer_email='zalewski.tomek03@gmail.com',
    description='żółwie sie kręcą',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'skz = prd1.turtle_tango:main'
        ],
    },
)
