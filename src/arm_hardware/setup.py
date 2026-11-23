from setuptools import setup

package_name = 'arm_hardware'

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
    description='Serial bridge from /arm_controller/joint_trajectory to the servo microcontroller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = arm_hardware.serial_bridge:main',
        ],
    },
)
