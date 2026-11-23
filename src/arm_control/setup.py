from setuptools import setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='ziadhu2005@gmail.com',
    description='Trajectory publisher, trajectory follower and slider GUI for the robotic arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_follower = arm_control.trajectory_follower:main',
            'trajectory_publisher = arm_control.trajectory_publisher:main',
            'slider_gui = arm_control.slider_gui:main',
        ],
    },
)
