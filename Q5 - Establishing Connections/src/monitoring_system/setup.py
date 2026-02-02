from setuptools import find_packages, setup

package_name = 'monitoring_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ## this line tells ROS 2 to install this .msg file, so that rosidl can generate .idl file for it
        # ('share/' + package_name + '/msg', ['msg/Coordinates.msg']),
        # ## switched to cmake pkg for msg
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanmitra',
    maintainer_email='goswamisanmitra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps = monitoring_system.gps:main',
            'differentiator = monitoring_system.differentiator:main',
            'checker = monitoring_system.checker:main',
        ],
    },
)
