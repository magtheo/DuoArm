from setuptools import find_packages, setup
<<<<<<< HEAD
import os, glob
=======
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a

package_name = 'map_and_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theomagnor',
    maintainer_email='theo@magnor.as',
=======
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asklindbraten',
    maintainer_email='asklindbraten@hotmail.com',
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapper = map_and_path.mapper:main',
            'path = map_and_path.path:main',
<<<<<<< HEAD


=======
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
        ],
    },
)
