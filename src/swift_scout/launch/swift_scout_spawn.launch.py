# Copyright 2024 Dhairya Shah

# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the “Software”), to deal 
# in the Software without restriction, including without limitation the rights to 
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
# of the Software, and to permit persons to whom the Software is furnished to do 
# so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


ld = LaunchDescription()

def get_num(context: LaunchContext, robot_num, use_rviz):
    print("function time babbeyy")
    num_str = context.perform_substitution(robot_num)
    rviz = context.perform_substitution(use_rviz)    

    spawned = None

    for i in range(int(num_str)):
        # print(spawned)
        x_pose = LaunchConfiguration('x_pose', default=f'-2')
        y_pose = LaunchConfiguration('y_pose', default=f'{i-1}')
        spawn_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('swift_scout'), 'launch'), 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'namespace': f'tb{i}',
                }.items()
            )
        
        # ld.add_action(spawn_robot)
        
        spawn_nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('swift_scout'), 'launch'), 'navigation2.launch.py')
                ),
                launch_arguments={
                    'namespace': f'tb{i}',
                }.items()
            )
        
        ld.add_action(spawn_robot)
        ld.add_action(spawn_nav2)
        
        if spawned is None:
            ld.add_action(spawn_robot)
            ld.add_action(spawn_nav2)
        else:
            spawn_turtlebot_first = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawned,
                    # on_exit=[spawn_robot]
                    on_exit=[spawn_robot, spawn_nav2],
                )
            )

            ld.add_action(spawn_turtlebot_first)
        spawned = spawn_robot

    for i in range(int(num_str)):
        spawn_rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('swift_scout'), 'launch'), 'rviz.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'namespace': f'tb{i}',
                    'use_rviz': use_rviz,
                }.items()
            )

        spawn_prev_first = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawned,
                    on_exit=[spawn_rviz],
                )
            )
        ld.add_action(spawn_prev_first)
        spawned = spawn_rviz


"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    package_dir = get_package_share_directory('swift_scout')


    robot_num_arg = DeclareLaunchArgument('robot_num', default_value='2')
    robot_num = LaunchConfiguration("robot_num", default='2')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False')
    use_rviz = LaunchConfiguration('use_rviz', default='False')

    function = OpaqueFunction(function=get_num, args=[LaunchConfiguration('robot_num'), LaunchConfiguration('use_rviz')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': PathJoinSubstitution([get_package_share_directory('turtlebot3_gazebo'),'worlds', 'turtlebot3_world.world']),
              'gui': 'true',
          }.items()
    )

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(package_dir, 'map', 'map.yaml'),
                     },],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ])
    
    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    goal_publisher_tb1 = Node(
        package='swift_scout',
        executable='scout',
        name='goal_pub_tb1',
        output='screen',
        arguments=['tb1']
    )

    goal_publisher_tb2 = Node(
        package='swift_scout',
        executable='scout',
        name='goal_pub_tb2',
        output='screen',
        arguments=['tb2']
    )

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    
    ld.add_action(robot_num_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(function)
    ld.add_action(gazebo)
    ld.add_action(goal_publisher_tb1)
    ld.add_action(goal_publisher_tb2)

    return ld