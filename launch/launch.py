from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from project4a.disc_robot import load_disc_robot

def generate_launch_description():

    robot = load_disc_robot('normal.robot')
    urdf = robot['urdf']
    robot_description = ParameterValue(urdf, value_type=str)

    default_rviz_config_path = "/home/jasdeep/ros2_ws/src/project4a/config/saved_rviz_config.rviz" 
    arg = DeclareLaunchArgument(name = 'rviz_config', default_value= default_rviz_config_path, description='path to the set configuration file of rviz')

    node1 = Node(package='project4a', executable='velocity_translator')  

    node2 = Node(package='project4a', executable='simulator')  

    node3 = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{'robot_description': robot_description}])

    node4 = Node(package="rviz2",executable="rviz2", arguments=['-d', LaunchConfiguration('rviz_config')]) 

    ld = LaunchDescription([arg,
        node1,  # Add the first Node action to the launch description
        node2,
        node3,
        node4,
    ])

    return ld  