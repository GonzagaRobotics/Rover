from setuptools import find_packages, setup

package_name = 'nav_sensors_antenna'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='damon',
    maintainer_email='57426668+BruhSoundEffectNumber2@users.noreply.github.com',
    description='ROS interface to nav sensors and antenna microcontroller.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_sensors_antenna = nav_sensors_antenna.main:main'
        ],
    },
)
