#!/usr/bin/env python3
"""
Launch file for UR3 MoveIt2 setup.
Starts robot_state_publisher + joint_state_publisher + MoveIt2 + RViz,
then adds table + marker-holder collision objects.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for UR3 MoveIt2 setup."""
    
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='IP address of UR3 robot',
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur3',
            description='UR robot type',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
    ]

    # ── Robot description (URDF from xacro) ──
    ur_type = LaunchConfiguration('ur_type')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']
        ),
        ' robot_ip:=xxx.yyy.zzz.www',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20',
        ' name:=ur',
        ' ur_type:=', ur_type,
        ' script_filename:=ros_control.urscript',
        ' input_recipe_filename:=rtde_input_recipe.txt',
        ' output_recipe_filename:=rtde_output_recipe.txt',
        ' prefix:=',
        ' joint_limit_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'joint_limits.yaml']
        ),
        ' kinematics_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml']
        ),
        ' physical_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'physical_parameters.yaml']
        ),
        ' visual_params:=',
        PathJoinSubstitution(
            [FindPackageShare('ur_description'), 'config', ur_type, 'visual_parameters.yaml']
        ),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description],
    )

    # ── MoveIt2 (move_group + RViz, no servo) ──
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_servo': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ── Collision objects after move_group fully loads ──
    scene_setup = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='ur3_motion_planning',
                executable='scene_publisher',
                name='scene_setup',
                output='screen',
            ),
        ],
    )
    
    # ── Isolate DDS domain so other students' robots don't interfere ──
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    ld = LaunchDescription(declared_arguments)
    ld.add_action(set_domain_id)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(ur_moveit_launch)
    ld.add_action(scene_setup)
    
    return ld
