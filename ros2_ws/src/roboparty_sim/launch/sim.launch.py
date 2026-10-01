# SPDX-License-Identifier: GPL-3.0
"""Start the simulated RPO: MuJoCo body + the real RoboParty inference_node.

    ros2 launch roboparty_sim sim.launch.py              # walk + wave + dances
    ros2 launch roboparty_sim sim.launch.py policy:=amp  # another brain
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    policy = LaunchConfiguration('policy').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context).lower() == 'true'

    # Brains (ONNX models, motions, policy configs) come unchanged from roboparty_inference.
    robot_dir = os.path.join(get_package_share_directory('roboparty_inference'), 'robots', 'rpo')
    policy_config = os.path.join(robot_dir, 'configs', f'{policy}.yaml')
    if not os.path.isfile(policy_config):
        raise FileNotFoundError(f'Policy config not found: {policy_config}')
    # Only the robot "hardware" config is ours: SIM motors and SIM IMU.
    robot_config = os.path.join(get_package_share_directory('roboparty_sim'), 'config', 'rpo_sim', 'robot.yaml')

    return [
        Node(package='roboparty_sim', executable='mujoco_sim', name='mujoco_sim',
             parameters=[{'viewer': viewer}], output='screen'),
        Node(
            package='roboparty_inference', executable='inference_node', name='inference_node',
            parameters=[policy_config, {
                'robot_name': 'rpo',
                'policy_name': policy,
                'robot_config': robot_config,
                'model_dir': os.path.join(robot_dir, 'models'),
                'motion_dir': os.path.join(robot_dir, 'motions'),
                'latent_dir': os.path.join(robot_dir, 'latents'),
            }],
            output='screen', respawn=True, respawn_delay=2.0),
        Node(package='roboparty_sim', executable='cmd_vel_mode', name='cmd_vel_mode', output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('policy', default_value='beyondmimic',
                              description='Policy config in roboparty_inference robots/rpo/configs'),
        DeclareLaunchArgument('viewer', default_value='true', description='Show the MuJoCo window'),
        OpaqueFunction(function=launch_setup),
    ])
