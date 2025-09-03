import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'arm'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output= 'screen')

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'rviz', 'view_bot.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    
    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[os.path.join(get_package_share_directory(package_name), "config", "controllers.yaml")],
            output="screen",
        ),
    
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"]  # Change to your controller name,
    )

    joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription(
        [rsp, 
         gazebo,
         spawn_entity,
         arm_controller,
         joint_state_broadcaster_controller,
        ]
    )