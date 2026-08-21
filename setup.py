from setuptools import find_packages, setup

package_name = 'ros2_project_el23eo'

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
    maintainer='Emmanuel Owoade',
    maintainer_email='eowoade05@gmail.com',
    description='ROS2/Nav2 TurtleBot package: colour-based object detection - an autonomous navigation project that patrols waypoints and reacts to a detected colour target.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_step = ros2_project_el23eo.first_step:main',
            'second_step = ros2_project_el23eo.second_step:main',
            'third_step = ros2_project_el23eo.third_step:main',
            'fourth_step = ros2_project_el23eo.fourth_step:main',
            'project_code = ros2_project_el23eo.project_code:main',
        ],
    },
)
