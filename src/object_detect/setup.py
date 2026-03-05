import glob
import os

from setuptools import find_packages, setup

package_name = 'object_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damon',
    maintainer_email='57426668+BruhSoundEffectNumber2@users.noreply.github.com',
    description='YOLO Object Detection and pose estimation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detect = object_detect.main:main',
        ],
    },
)
