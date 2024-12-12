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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Declare number of robots to spawn
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to spawn in the environment'
    )
    
    # Declare RViz option
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Enable RViz visualization for each robot'
    )

    # Launch configurations
    num_robots = LaunchConfiguration('num_robots')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot spawn locations
    robot_locations = [
        {'x_pose': '-1.5', 'y_pose': '-0.4', 'z_pose': 0.01},
        {'x_pose': '-1.5', 'y_pose': '0.4', 'z_pose': 0.01},
        {'x_pose': '-1.5', 'y_pose': '0.9', 'z_pose': 0.01},
        {'x_pose': '-1.5', 'y_pose': '-0.9', 'z_pose': 0.01},
        {'x_pose': '-1.5', 'y_pose': '0.0', 'z_pose': 0.01},
        {'x_pose': '-2.1', 'y_pose': '0.0', 'z_pose': 0.01},
    ]

    # Directories and configurations
    package_dir = get_package_share_directory('swift_scout')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf = os.path.join(package_dir, 'urdf', 'turtlebot3_waffle.urdf')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'multi_nav2_default_view.rviz')
    params_file = os.path.join(package_dir, 'param', 'nav2_params.yaml')

    # Launch description
    ld = LaunchDescription()
    ld.add_action(num_robots_arg)
    ld.add_action(use_rviz_arg)

    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': PathJoinSubstitution([package_dir, 'worlds', 'person.world']),
            'gui': 'true',
        }.items()
    )
    ld.add_action(gazebo)

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(package_dir, 'map', 'map.yaml')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
    )
    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)

    # Robot spawning and node setup
    last_spawn_action = None
    num_robots_count = int(os.environ.get('NUM_ROBOTS', '2'))  # Number of robots from environment variable

    for i, robot_location in enumerate(robot_locations):
        # Limit the number of robots to the specified `num_robots`
        if i >= num_robots_count:
            break

        namespace = f'tb{i + 1}'

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'publish_frequency': 10.0}],
            arguments=[urdf],
        )

        # Spawn the robot in Gazebo
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(package_dir, 'model', 'turtlebot3_waffle', 'model.sdf'),
                '-entity', namespace,
                '-robot_namespace', f'/{namespace}',
                '-x', robot_location['x_pose'],
                '-y', robot_location['y_pose'],
                '-z', str(robot_location['z_pose']),
                '-unpause',
            ],
            output='screen',
        )

        # Bringup Navigation for the robot
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'namespace': f'/{namespace}',
                'use_namespace': 'True',
                'map': '',
                'map_server': 'False',
                'params_file': params_file,
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                'autostart': 'true',
                'use_sim_time': 'true',
                'log_level': 'warn'
            }.items()
        )

        # Initial pose setup
        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-1', '--qos-reliability', 'reliable',
                f'/{namespace}/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                '{'
                '  "header": {'
                '    "frame_id": "map"'
                '  },'
                '  "pose": {'
                '    "pose": {'
                '      "position": {'
                f'        "x": {robot_location["x_pose"]},'
                f'        "y": {robot_location["y_pose"]},'
                '        "z": 0.0'
                '      },'
                '      "orientation": {'
                '        "x": 0.0,'
                '        "y": 0.0,'
                '        "z": 0.0,'
                '        "w": 1.0'
                '      }'
                '    },'
                '    "covariance": [0.0]*36'
                '  }'
                '}'
            ],
            output='screen'
        )

        # Add all nodes for this robot
        ld.add_action(robot_state_publisher)
        ld.add_action(spawn_robot)
        ld.add_action(bringup_cmd)
        ld.add_action(initial_pose_cmd)

        # Track the last spawn action
        last_spawn_action = spawn_robot

    # RViz Node (Launched after the last robot spawn)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz_config': rviz_config_file
        }.items(),
        condition=IfCondition(use_rviz)  # Ensure RViz is launched only if use_rviz is true
    )

    # Register RViz launch to occur after the last robot is spawned
    if last_spawn_action:
        rviz_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_spawn_action,
                on_exit=[rviz_cmd]
            )
        )
        ld.add_action(rviz_event)

    return ld
