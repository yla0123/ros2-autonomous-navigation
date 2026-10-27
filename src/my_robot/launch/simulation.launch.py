# Bridge
from launch import LaunchDescription
from launch_ros.actions import Node

# Nodes
from ament_index_python.packages import get_package_share_directory
import os

# Gazebo
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    package_share = get_package_share_directory("my_robot")
    ekf_config = os.path.join(package_share, "config", "ekf.yaml")
    path_follower_config = os.path.join(get_package_share_directory('my_robot'), 'config', 'path_follower.yaml')
    joint_state_gz_topic = ("/world/robot_world/model/mobile_robot/joint_state")

    bridge = Node(
        package = "ros_gz_bridge",
        executable ="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments = [            
            # Gazebo -> ROS2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            joint_state_gz_topic
            + "@sensor_msgs/msg/JointState[gz.msgs.Model",

            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            "/model/mobile_robot/odometry"
            + "@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            # ROS2 -> Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",],
        remappings=[
            (joint_state_gz_topic,"/joint_states"),
            ("/model/mobile_robot/odometry","/gazebo_odom"),],
    )

    odometry_node = Node(
        package = "my_robot",
        executable = "odometry_node",
        name = "odometry_node",
        output = "screen",
        parameters = [{"use_sim_time": True}],
    )
    lidar_node = Node(
        package='my_robot',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
    )
    path_follower_node = Node(
        package='my_robot',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[path_follower_config],
    )
    ekf_node = Node(        
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    evaluation_node = Node(
        package='my_robot',
        executable='evaluation_node',
        name='evaluation_node',
        output='screen',
        parameters=[path_follower_config],
    )

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_file = os.path.join(package_share, "worlds", "robot_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments = {"gz_args": f"-r {world_file}"}.items()
    )


    return LaunchDescription([gazebo, bridge, odometry_node, ekf_node, lidar_node, path_follower_node, evaluation_node])