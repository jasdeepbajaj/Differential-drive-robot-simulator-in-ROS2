from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from project4a.disc_robot import load_disc_robot
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import sys

def generate_launch_description():
    """
    Generates a LaunchDescription for launching the robot simulation.

    Returns:
        LaunchDescription: The launch description for the robot simulation.
    """

    # Declare launch arguments
    arg1 = DeclareLaunchArgument(name='robot_name', default_value='normal.robot', description='Name of the robot')
    robot_name = [arg for arg in sys.argv if arg.startswith("robot_name:=")][0].split(':=')[1]

    # Load robot description from disc_robot module
    robot_path = f"/home/jasdeep/ros2_ws/src/project4a/robot/{robot_name}" 
    robot = load_disc_robot(robot_path)
    
    urdf = robot['urdf']
    robot_description = ParameterValue(urdf, value_type=str)

    # Extract wheel-related parameters from the robot configuration
    wheel_distance = robot['wheels']['distance']
    error_variance_left = robot['wheels']['error_variance_left']
    error_variance_right = robot['wheels']['error_variance_right']
    error_update_rate = robot['wheels']['error_update_rate']

    # Declare launch argument for RViz configuration file
    default_rviz_config_path = "/home/jasdeep/ros2_ws/src/project4a/config/saved_rviz_config.rviz" 
    arg2 = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path, description='Path to the RViz configuration file')

    # Declare launch argument for bag input
    arg3 = DeclareLaunchArgument('bag_input')

    # Generate paths for bag input, bag file, and recorded bag file
    bag_file_path = Command(["echo /home/jasdeep/ros2_ws/src/project4a/bags/project4a-bags/", LaunchConfiguration('bag_input')])
    bag_record_file_path = Command(["echo /home/jasdeep/ros2_ws/src/project4a/bags/recordings/", LaunchConfiguration('bag_input'), "-out"])

    # Define ExecuteProcess actions for playing and recording bags
    ep1 = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_file_path], shell=True, name='ros2_bag_play')
    ep2 = ExecuteProcess(cmd=['ros2', 'bag', 'record', '/clicked_point', '/cmd_vel', '/events/read_split', '/goal_pose', '/initial_pose',
                              '/joint_states', '/parameter_events', '/robot_description', '/rosout', '/tf', '/tf_static', '/vl', '/vr', '-o',
                              bag_record_file_path], shell=True, name='ros2_bag_record')

    # Define VelocityTranslator node with wheel_distance parameter
    velcoity_translator_node = Node(package='project4a', executable='velocity_translator', parameters=[{'wheel_distance': wheel_distance}])

    # Define Simulator node with wheel-related parameters
    simulator_node = Node(package='project4a', executable='simulator', parameters=[{'wheel_distance': wheel_distance},
                                                                                   {'error_variance_left': error_variance_left},
                                                                                   {'error_variance_right': error_variance_right},
                                                                                   {'error_update_rate': error_update_rate}])

    # Define Robot State Publisher node with robot_description parameter
    robot_publisher_node = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{'robot_description': robot_description}])

    # Define RViz node with the specified RViz configuration file
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=['-d', LaunchConfiguration('rviz_config')])

    # Register an event handler to terminate the launch on process exit
    # event_handler = OnProcessExit(target_action=ep1, on_exit=[EmitEvent(event=Shutdown())])
    # terminate_at_end = RegisterEventHandler(event_handler)

    # Define the LaunchDescription
    ld = LaunchDescription([arg1,
                            arg2,
                            arg3,
                            ep1,
                            ep2,
                            velcoity_translator_node,
                            simulator_node,
                            robot_publisher_node,
                            rviz_node])

    return ld  
