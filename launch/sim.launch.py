"""Gazebo + UR3e + gripper + camera + MoveIt 2 + RViz.

    ros2 launch ur3_llm_control sim.launch.py gazebo_gui:=false launch_rviz:=false
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from ur3_llm_control.scene import Scene
from ur3_llm_control.world_gen import make_world


def setup(context):
    ur = LaunchConfiguration('ur_type').perform(context)
    me = get_package_share_directory('ur3_llm_control')
    mc = get_package_share_directory('ur_moveit_config')

    # 1. world
    world = os.path.join(tempfile.gettempdir(), 'ur3_llm.world')
    with open(world, 'w') as f:
        f.write(make_world(Scene(os.path.join(me, 'config', 'scene.yaml'))))

    # 2. robot (UR3e + gripper)
    xacro = FindExecutable(name='xacro')
    urdf = Command([xacro, ' ', os.path.join(me, 'urdf', 'ur_gripper.urdf.xacro'),
                    ' ur_type:=', ur, ' sim_gazebo:=true',
                    ' simulation_controllers:=', os.path.join(me, 'config', 'controllers.yaml'),
                    ' initial_positions_file:=', os.path.join(me, 'config', 'initial_positions.yaml')])
    desc = {'robot_description': ParameterValue(urdf, value_type=str)}
    srdf = Command([xacro, ' ', os.path.join(me, 'srdf', 'ur_gripper.srdf.xacro')])
    sem = {'robot_description_semantic': ParameterValue(srdf, value_type=str)}

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='both', parameters=[{'use_sim_time': True}, desc])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'gui': LaunchConfiguration('gazebo_gui'), 'world': world}.items())
    # timeout 120s vi gzserver tren WSL khoi dong cham
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'ur', '-topic', 'robot_description', '-timeout', '120'],
                 output='screen')

    # 3. controller: joint_state_broadcaster roi den canh tay + gripper
    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', '/controller_manager'])
    jsb = spawner('joint_state_broadcaster')
    after = RegisterEventHandler(OnProcessExit(
        target_action=jsb,
        on_exit=[spawner('joint_trajectory_controller'), spawner('gripper_controller')]))

    # 4. MoveIt 2 (move_group) - cau hinh lay tu ur_moveit_config, URDF/SRDF co gripper
    ctrl = load_yaml('ur_moveit_config', 'config/controllers.yaml')
    ctrl['scaled_joint_trajectory_controller']['default'] = False
    ctrl['joint_trajectory_controller']['default'] = True
    ompl = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1}}
    ompl['move_group'].update(load_yaml('ur_moveit_config', 'config/ompl_planning.yaml'))
    kin = os.path.join(mc, 'config', 'kinematics.yaml')
    lim = {'robot_description_planning': load_yaml('ur_moveit_config', 'config/joint_limits.yaml')}
    params = [desc, sem, kin, lim, ompl, {
        'moveit_simple_controller_manager': ctrl,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'use_sim_time': True}]
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      output='screen', parameters=params)
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2_moveit', output='log',
                arguments=['-d', os.path.join(mc, 'rviz', 'view_robot.rviz')],
                parameters=[desc, sem, kin, lim, ompl, {'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('launch_rviz')))

    return [gazebo, rsp, spawn, jsb, after, move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e', choices=['ur3', 'ur3e']),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
