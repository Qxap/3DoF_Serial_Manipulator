from setuptools import find_packages, setup

from glob import glob
import os


package_name = 'ms2r1p'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'model'), glob("model/*.*")),
        (os.path.join("share", package_name,'dxfs'), glob("dxfs/*.*")),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools', 'ezdxf', 'pyyaml'],
    zip_safe=True,
    maintainer='helloworld',
    maintainer_email='helloworld@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ms2r1p_state_publisher= ms2r1p.ms2r1p_state_publisher:main',
            #'dxf_parser_node= ms2r1p.dxf_exporter_node:main',
            'dxf_parser_node= ms2r1p.dxf_exporter_node_v2:main',
            'direct_kinematics= ms2r1p.direct_kinematics_node:main',
            'inverse_kinematics= ms2r1p.inverse_kinematics_node:main',
            #'goal_pose_traducer= ms2r1p.goal_pose_traducer:main',
            'trajectory_planner_node= ms2r1p.trajectory_planner_node:main',
            'trajectory_follower= ms2r1p.trajectory_follower:main',
        ],
    },
)
