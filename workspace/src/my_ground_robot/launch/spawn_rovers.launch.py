import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_ground_robot'
    
    # 1. MURI
    walls_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/complex_walls.sdf')
    with open(walls_file, 'r') as infp: walls_desc = infp.read()
    
    spawn_walls = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'complex_walls', '-string', walls_desc, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # 2. ROVERS
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/rover.urdf.xacro')
    
    rover1_xml = xacro.process_file(xacro_file, mappings={'robot_name': 'rover_1', 'color': 'Olive'}).toxml()
    spawn_r1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rover_1', '-string', rover1_xml, '-x', '2.0', '-y', '2.0', '-z', '0.5'],
        output='screen'
    )

    emergency_xml = xacro.process_file(xacro_file, mappings={'robot_name': 'rover_emergency', 'color': 'Desert'}).toxml()
    spawn_r2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rover_emergency', '-string', emergency_xml, '-x', '2.0', '-y', '-3.0', '-z', '0.5'],
        output='screen'
    )

    # 3. BRIDGE UNICO (Inclusa Camera Drone)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            # ROVER 1
            '/model/rover_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/rover_1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/rover_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # ROVER EMERGENCY
            '/model/rover_emergency/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/rover_emergency/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/rover_emergency/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # --- CAMERA DRONE ---
            # Bridge unidirezionale da Gazebo a ROS
            '/drone/downward_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen',
        remappings=[
            ('/model/rover_1/cmd_vel', '/rover_1/cmd_vel'),
            ('/model/rover_1/scan', '/rover_1/scan'),
            ('/model/rover_1/odometry', '/rover_1/odom'),
            
            ('/model/rover_emergency/cmd_vel', '/rover_emergency/cmd_vel'),
            ('/model/rover_emergency/scan', '/rover_emergency/scan'),
            ('/model/rover_emergency/odometry', '/rover_emergency/odom'),
            
            # Remap Camera
            ('/drone/downward_cam/image_raw', '/drone/camera'),
        ]
    )

    return LaunchDescription([spawn_walls, spawn_r1, spawn_r2, bridge])
