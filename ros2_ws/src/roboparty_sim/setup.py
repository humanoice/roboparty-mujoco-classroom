from glob import glob
from setuptools import setup

package_name = 'roboparty_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config/rpo_sim', glob('config/rpo_sim/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'mujoco_sim = roboparty_sim.mujoco_sim:main',
            'cmd_vel_mode = roboparty_sim.cmd_vel_mode:main',
            'party = roboparty_sim.party:main',
            'remote = roboparty_sim.remote:main',
        ],
    },
)
