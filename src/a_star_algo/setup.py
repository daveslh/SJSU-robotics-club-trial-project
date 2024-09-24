from setuptools import find_packages, setup

package_name = 'a_star_algo'

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
    maintainer='David S. Huang',
    maintainer_email='david.sl.huang@gmail.com',
    description='Robotics Club Trial Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'create_map = a_star_algo.map_node:main',
                'create_goal_and_start = a_star_algo.goal_and_start_node:main',
                'create_path_viz = a_star_algo.path_node:main',
        ],
    },
)
