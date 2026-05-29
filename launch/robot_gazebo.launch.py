from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_path('slam_optimization_pkg'),
        'urdf',
        'main.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}   
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            '-d',
            os.path.join(
                get_package_share_path('slam_optimization_pkg'),
                'rviz',
                'config.rviz'
            )
        ],
        parameters=[{'use_sim_time': True}] 
    )

    world_path = os.path.join(
        get_package_share_directory('slam_optimization_pkg'),
        'world',
        'home_plan.world'
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'loc_robot'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_node,                 
        robot_state_publisher_node,  
        spawn_entity,
        rviz_node,
        

    ])
