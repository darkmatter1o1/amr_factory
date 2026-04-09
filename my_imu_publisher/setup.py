from setuptools import setup

package_name = 'my_imu_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to publish IMU data from an Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = my_imu_publisher.imu_publisher:main',
            'goal = my_imu_publisher.goal:main',
            'light = my_imu_publisher.light:main',
            'amcl_pose = my_imu_publisher.amcl_pose:main',
            'battery = my_imu_publisher.battery:main',
            'thick = my_imu_publisher.thick:main',
            'logger = my_imu_publisher.logger:main',
            'alert = my_imu_publisher.alert:main'
        ],
    },
)

