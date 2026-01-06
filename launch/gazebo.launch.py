from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, AppendEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_description')

    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    xacro_file = os.path.join(pkg_path, 'urdf', 'URDF.xacro')
    robot_description = os.popen(f'xacro {xacro_file}').read()

    slam_launch_path = os.path.join(
        get_package_share_directory('urdf_description'),
        'launch',
        'online_async_launch.py'
    )
    
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')

    # 2. Define the Include action
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': slam_params_file
        }.items()
    )

    rviz_launch_path = os.path.join(
        get_package_share_directory('urdf_description'),
        'launch',
        'display.launch.py'
    )
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([

        set_resource_path,
        include_slam,
        include_rviz,

        # Start Gazebo
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', 'empty.sdf'], output='screen'),

        # publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
        ),


        # Spawn robot
        TimerAction(
            period=5.0,  
            actions=[
                Node(
                    package='ros_gz_sim', 
                    executable='create',
                    arguments=[
                        '-world', 'empty',
                        '-name', 'URDF',
                        '-topic', 'robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.5'
                    ],
                    output='screen'
                )
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
            ],
            output='screen'
        ),
                
    ])