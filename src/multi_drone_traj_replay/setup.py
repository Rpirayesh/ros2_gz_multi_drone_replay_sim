from setuptools import setup
import os
from glob import glob

package_name = 'multi_drone_traj_replay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add your models
        (os.path.join('share', package_name, 'models'), glob('models/**/*.sdf', recursive=True)),
        # Add your CSV file
        (os.path.join('share', package_name), ['traj_eval_Ours.csv']),
        # Also add launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Replay multiple drones from trajectory CSV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_replay_node = multi_drone_traj_replay.traj_replay_node:main',
        ],
    },
)
