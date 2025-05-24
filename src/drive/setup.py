from setuptools import find_packages, setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.2.0',
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
    description='Driving system interop.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive = drive.drive:main',
        ],
    },
)
