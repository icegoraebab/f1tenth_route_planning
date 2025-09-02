from setuptools import setup
from glob import glob

package_name = 'f1tenth_route_planning'

setup(
    name=package_name,  # ← 언더스코어 유지!
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='A* global planner + pure pursuit (map→base_link)',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'global_planner = f1tenth_route_planning.global_planner:main',
            'pure_pursuit  = f1tenth_route_planning.pure_pursuit_controller:main',
            'odom2map_tf = f1tenth_route_planning.odom_to_map_tf:main',
        ],
    },
)
