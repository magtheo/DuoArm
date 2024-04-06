from setuptools import find_packages, setup
import os
import glob

package_name = 'controll_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theo',
    maintainer_email='theo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_window = controll_center.command_window:main',
            'command_executor = controll_center.command_executor:main',
            'display = controll_center.display:main',
            'action_controller = controll_center.action_controller:main',
            'joystick_controller = controll_center.joystick_controller:main',
         
        ],
    },
)
