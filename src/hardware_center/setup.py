from setuptools import find_packages, setup
import os, glob

package_name = 'hardware_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asklindbraten',
    maintainer_email='asklindbraten@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = hardware_center.motor_control:main',
            'hardware_interface_controller = hardware_center.hardware_interface_controller:main',
            'gpio_controller = hardware_center.gpio_controller:main',

        ],
    },
)
