from setuptools import setup

package_name = 'drone_detection_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detection_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aerospace_user',
    maintainer_email='example@example.com',
    description='Drone follower with YOLO-based detection tracking for ROS 2 Humble',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_detection_follower = drone_detection_follower.drone_detection_follower:main',
        ],
    },
)
