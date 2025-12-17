import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Definisce il nome del pacchetto dove si trovano i file urdf/sdf
    pkg_name = 'my_ground_robot'
    
    # ---------------------------------------------------------
    # 1. SPAWN DELL'AMBIENTE (MURI/LABIRINTO)
    # ---------------------------------------------------------
    # percorso completo al file 'complex_walls.sdf'
    walls_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/complex_walls.sdf')
    
    # Legge il contenuto del file SDF in una variabile stringa
    # perché il nodo 'create' di ros_gz_sim accetta il contenuto XML come stringa.
    with open(walls_file, 'r') as infp: walls_desc = infp.read()
    
    # Definizione nodo walls
    spawn_walls = Node(
        package='ros_gz_sim', 
        executable='create', # L'eseguibile specifico per spawnare entità
        arguments=[
            '-name', 'complex_walls',  
            '-string', walls_desc,     # Il contenuto del file SDF letto sopra
            '-x', '0', '-y', '0', '-z', '0' # Posizione di spawn 
        ],
        output='screen' # Mostra i log nel terminale
    )

    # ---------------------------------------------------------
    # 2. SPAWN DEI ROVER
    # ---------------------------------------------------------
    # Percorso del file principale .xacro dei rover
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/rover.urdf.xacro')
    
    # --- ROVER 1 ---
    # Processa il file xacro sostituendo i parametri:
    # 'robot_name' diventa 'rover_1' e 'color' diventa 'Olive'.
    # .toxml() converte l'oggetto processato in una stringa XML pura (URDF finale).
    rover1_xml = xacro.process_file(xacro_file, mappings={'robot_name': 'rover_1', 'color': 'Olive'}).toxml()
    
    # Nodo per spawnare il primo rover
    spawn_r1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'rover_1',      # Nome univoco in Gazebo
            '-string', rover1_xml,   # L'URDF generato dallo xacro
            '-x', '2.0', '-y', '2.0', '-z', '0.5' # Coordinate iniziali (x=2, y=2)
        ],
        output='screen'
    )

    # --- ROVER EMERGENCY (ROVER 2) ---
    # Processa lo STESSO file xacro ma con parametri diversi (Nome: rover_emergency, Colore: Desert).
    # Questo evita di avere due file URDF separati per robot quasi identici.
    emergency_xml = xacro.process_file(xacro_file, mappings={'robot_name': 'rover_emergency', 'color': 'Desert'}).toxml()
    
    # Nodo per spawnare il secondo rover
    spawn_r2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'rover_emergency',
            '-string', emergency_xml,
            '-x', '2.0', '-y', '-3.0', '-z', '0.5' # Coordinate iniziali diverse (x=2, y=-3)
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # 3. BRIDGE (PONTE) ROS 2 <-> GAZEBO
    # ---------------------------------------------------------
    # Traduce i messaggi di Gazebo (Gz Transport) in messaggi ROS 2 e viceversa.
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        arguments=[
            # SINTASSI ARGOMENTI: Topic_Gazebo@Tipo_ROS@Tipo_Gazebo
            
            # --- ROVER 1 BRIDGES ---
            # Cmd_vel: ROS -> Gazebo (per muovere il robot)
            '/model/rover_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Scan: Gazebo -> ROS (dati del Lidar)
            '/model/rover_1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # Odometry: Gazebo -> ROS (posizione del robot)
            '/model/rover_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # --- ROVER EMERGENCY BRIDGES ---
            # Stessi topic del primo rover, ma puntati sul namespace /model/rover_emergency/
            '/model/rover_emergency/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/rover_emergency/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/rover_emergency/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # --- CAMERA DRONE ---
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
            
            # Remap Camera Drone
            ('/drone/downward_cam/image_raw', '/drone/camera'),
        ]
    )

    # Robot State Publisher node
    rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': rover1_xml, 
                'use_sim_time': True
            }]
        )

    return LaunchDescription([
        spawn_walls, 
        spawn_r1, 
        spawn_r2, 
        bridge, 
        rsp_node
    ])
