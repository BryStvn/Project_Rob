from setuptools import find_packages, setup

package_name = 'base_lidar_move'

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
    maintainer='brayanpadilla',
    maintainer_email='brayanpadilla198@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_lidar_move = base_lidar_move.base_lidar_move:main',
            'lidar_tf_broadcaster = my_robot_tf2.lidar_tf_broadcaster:main',
        ],
    },
)
