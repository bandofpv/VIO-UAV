from setuptools import setup

package_name = 'mavrospy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/sim.rviz']),
        ('share/' + package_name + '/launch', ['launch/rviz.py']),
        ('share/' + package_name + '/launch', ['launch/mocap.py']),
        ('share/' + package_name + '/launch', ['launch/outdoor.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_sim.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Bernas',
    maintainer_email='andrewrbernas@gmail.com',
    description='ROS node to interact with MAVROS for basic UAV control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_py = mavrospy.square:main',
            'square_head_py = mavrospy.square_head:main',
            'circle_py = mavrospy.circle:main',
            'circle_head_py = mavrospy.circle_head:main',
            'figure8_py = mavrospy.figure8:main',
            'figure8_head_py = mavrospy.figure8_head:main',
            'spiral_py = mavrospy.spiral:main',
            'spiral_head_py = mavrospy.spiral_head:main',
            'persistent_excitation_py = mavrospy.persistent_excitation:main',
            'fake_vio_gps_py = mavrospy.fake_vio_gps:main',
            'pose_to_path_py = mavrospy.pose_to_path:main',
        ],
    },
)

