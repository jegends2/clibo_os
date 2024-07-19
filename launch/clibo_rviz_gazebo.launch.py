import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name='clibo_os'
    gazebo_package='gazebo_ros'

    pkg_dir = get_package_share_directory(package_name)
    gazebo_pkg_dir = get_package_share_directory(gazebo_package)


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_dir,'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz2 = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'view_bot.rviz')]
    )

    gazebo_params_file = os.path.join(pkg_dir, 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg_dir, 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'world':os.path.join(pkg_dir, 'worlds', 'office.world'),
                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
                }.items()             

             )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen')
    
    teleop_cmd = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )


    return LaunchDescription([
        rsp,
        rviz2,
        spawn_entity,
        gazebo,
        teleop_cmd

    ])