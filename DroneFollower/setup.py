from setuptools import setup

package_name = 'drone_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_follower']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Drone follower node: follows detected drone in Gazebo using pose topics and YOLO detections.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_follower_node = drone_follower.drone_follower_node:main',
        ],
    },
)
