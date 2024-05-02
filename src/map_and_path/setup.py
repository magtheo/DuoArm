from setuptools import find_packages, setup

package_name = 'map_and_path'

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
    maintainer='theomagnor',
    maintainer_email='theo@magnor.as',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapper = map_and_path.mapper:main',
            'path = map_and_path.path:main',


        ],
    },
)
