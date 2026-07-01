import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'arm_description'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robot',
                    '-topic', '/robot_description',
                    '-x', '0.0', # set these as per your world
                    '-y', '0.0',
                    '-z', '0.0',
                    # 'R', '0.0',
                    # 'P', '0.0',
                    # 'Y', '1.57'
                ],
                output='screen'
            )
        ]
    )
    
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"]
    )

    joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
            rsp, 
            spawn,
            arm_controller,
            joint_state_broadcaster_controller,
        ])