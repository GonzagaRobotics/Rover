import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'calibrations'), glob('calibrations/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damon',
    maintainer_email='57426668+BruhSoundEffectNumber2@users.noreply.github.com',
    description='Sends camera data to multiple nodes.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision = vision.main:main',
        ],
    },
)
