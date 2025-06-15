from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------Jetson TX2-NX Start---------------------")
def generate_launch_description():
    
    
    urdf_path = os.path.join(get_package_share_directory('robot_car'),'urdf','my_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)#解析xacro檔轉成urdf檔
    rviz_config_path = os.path.join(get_package_share_directory('robot_car'),'rviz','selfcar.rviz')
    rplidar_ros_path = os.path.join(get_package_share_directory('rplidar_ros'),'launch','rplidar_c1_launch.py')

    # gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui')
    # model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
    #                                   description='Absolute path to robot urdf file')
    # rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
    #                                  description='Absolute path to rviz config file')
    # pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
    #                                         description='Whether to publish the tf from the original odom to the base_footprint')

    # robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
    #                                    value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('turtlesim_test'),
        'param',
        'imu_filter_param.yaml'
    ) 

    driver_node = Node(
        package='turtlesim_test',
        executable='ps4_turtle',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    ps4_joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        driver_node,
        imu_filter_node,
        ps4_joy_node
    ])
