from setuptools import find_packages, setup

package_name = 'controls'

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
    maintainer='jacob',
    maintainer_email='jakanji@calpoly.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
            'joy_input = controls.joy_input:main',
            'cmd_vel_to_manual_control = controls.cmd_vel_to_manual_control:main',
            'joy_to_manual_control = controls.joy_to_manual_control_node:main',
        ],
    },
)
