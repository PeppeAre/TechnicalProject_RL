import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math
import threading
import sys
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission_node')

        # QoS
        qos_px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_status = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_px4)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_px4)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_px4)
        
        self.r1_target_pub = self.create_publisher(Point, '/rover_1/target', 10)
        self.r2_target_pub = self.create_publisher(Point, '/rover_emergency/target', 10)

        # Subscribers
        self.create_subscription(String, '/rover_1/status', self.r1_status_cb, qos_status)
        self.create_subscription(String, '/rover_emergency/status', self.r2_status_cb, qos_status)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_px4)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odom_callback, qos_px4)

        # Stato
        self.current_position = [0.0, 0.0, 0.0]
        self.offboard_setpoint_counter_ = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0 
        
        # Logica Flotta
        self.r1_broken = False
        self.r2_broken = False
        self.mission_aborted = False 
        self.mission_active_rover = 1 
        self.home_pos_r2 = Point(x=2.0, y=-3.0, z=0.0)
        
        # Logica Interactive
        self.drone_arrived = False   # Flag impostato dal timer quando il drone arriva
        self.mission_started = False # Flag impostato dall'utente (start/no)

        # Timer Antirimbalzo Status
        self.last_r1_change = 0.0
        self.last_r2_change = 0.0
        self.debounce = 2.0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True 
        self.input_thread.start()

    def vehicle_status_callback(self, msg): pass
    def vehicle_odom_callback(self, msg): self.current_position = msg.position
    def get_time(self): return self.get_clock().now().nanoseconds / 1e9

    # --- LOGICA USER INPUT ---
    def user_input_loop(self):
        # Attesa inziale arming
        while self.offboard_setpoint_counter_ < 150: time.sleep(0.5)
        
        print("\n-------------------------------------------------")
        print(">>> SYSTEM ACTIVE.")
        print("-------------------------------------------------\n")
        sys.stdout.flush()

        while rclpy.ok():
            try:
                # 1. Chiede Coordinate
                print("\n[COMMAND] Insert Napoli Drone coordinates:")
                in_x = input("Target X: ")
                in_y = input("Target Y: ")
                
                self.target_x = float(in_x)
                self.target_y = float(in_y)
                
                # Resetta stati per il nuovo volo
                self.drone_arrived = False
                self.mission_started = False
                self.mission_aborted = False
                
                print(f">>> Drone flying to ({self.target_x}, {self.target_y})...")
                sys.stdout.flush()

                # 2. Attesa Arrivo Drone (Polling sul flag impostato dal timer)
                while not self.drone_arrived:
                    time.sleep(0.5)


                # 3. Drone Arrivato: Chiede Conferma
                print("\n-------------------------------------------------")
                print(f">>> DRONE ARRIVED IN ({self.target_x}, {self.target_y}).")
                print(">>> Scan area via camera.")
                cmd = input("[COMMAND] Do you want to start a mission? (start / no): ").strip().lower()
                
                if cmd == 'start':
                    print(">>> MISSION CONFIRMED. ROVER ARE COMING.")
                    self.mission_started = True # Questo sblocca l'invio al rover nel timer
                    
                else:
                    print(">>> MISSION CANCELED. Waiting for new coordinates.")
                    # Non settiamo mission_started, quindi il rover non parte.
                    # Il loop ricomincia e chiede nuove X, Y.

            except ValueError:
                print("! Error numeric input.")
            except EOFError:
                break

    # --- LOGICA FDIR (Fault Detection) ---
    def check_global_failure(self):
        if self.r1_broken and self.r2_broken and not self.mission_aborted:
            self.get_logger().error("\n!!! CRITICAL: BOTH ROVERS BROKEN. RETURN TO BASE !!!")
            self.mission_aborted = True
            self.mission_started = False # Stop invio ai rover
            self.target_x = 0.0
            self.target_y = 0.0

    def r1_status_cb(self, msg):
        # Ignora status se la missione non è partita
        if not self.mission_started: return

        now = self.get_time()
        is_broken = (msg.data == "BROKEN")

        if is_broken and not self.r1_broken:
            if (now - self.last_r1_change) > self.debounce:
                self.get_logger().error("!!! ALARM: ROVER 1 BROKEN! EMERGENCY ROVER ACTIVE !!!")
                self.r1_broken = True
                self.last_r1_change = now
                self.check_global_failure()
                
                if not self.mission_aborted and not self.r2_broken:
                    self.mission_active_rover = 2
                    # Invia subito target a R2
                    self.r2_target_pub.publish(Point(x=self.target_x, y=self.target_y, z=0.0))

        elif not is_broken and self.r1_broken:
            if (now - self.last_r1_change) > self.debounce:
                self.get_logger().info(">>> ROVER 1 REPAIRED. EMERGENCY ROVER COMING TO BASE.")
                self.r1_broken = False
                self.last_r1_change = now
                self.mission_active_rover = 1
                self.mission_aborted = False
                
                self.r2_target_pub.publish(self.home_pos_r2)
                self.r1_target_pub.publish(Point(x=self.target_x, y=self.target_y, z=0.0))

    def r2_status_cb(self, msg):
        if not self.mission_started: return
        now = self.get_time()
        is_broken = (msg.data == "BROKEN")

        if is_broken and not self.r2_broken:
            if (now - self.last_r2_change) > self.debounce:
                self.get_logger().error("! EMERGENCY ROVER BROKEN.")
                self.r2_broken = True
                self.last_r2_change = now
                self.check_global_failure()

        elif not is_broken and self.r2_broken:
            if (now - self.last_r2_change) > self.debounce:
                self.get_logger().info(">>> EMERGENCY ROVER REPAIRED.")
                self.r2_broken = False
                self.last_r2_change = now
                self.mission_aborted = False
                if self.r1_broken:
                     self.mission_active_rover = 2
                     self.r2_target_pub.publish(Point(x=self.target_x, y=self.target_y, z=0.0))

    # --- COMUNICAZIONE PX4 ---
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [y, x, z] # Fix NED
        msg.yaw = 0.0 
        self.trajectory_setpoint_publisher_.publish(msg)

    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.target_x, self.target_y, self.target_z)

        # Calcolo Distanza per check arrivo
        curr_x = self.current_position[1]
        curr_y = self.current_position[0]
        dist = math.sqrt((self.target_x - curr_x)**2 + (self.target_y - curr_y)**2)

        # Se sono arrivato, avviso il thread di input (una volta sola)
        if dist < 0.5 and not self.drone_arrived and self.offboard_setpoint_counter_ > 200:
            self.drone_arrived = True # Sblocca la domanda "start/no"

        # INVIO TARGET AI ROVER (Solo se l'utente ha detto START)
        if self.mission_started and not self.mission_aborted:
            # Invio periodico (1Hz) per sicurezza
            if self.offboard_setpoint_counter_ % 10 == 0:
                target_msg = Point(x=self.target_x, y=self.target_y, z=0.0)
                
                if self.mission_active_rover == 1 and not self.r1_broken:
                    self.r1_target_pub.publish(target_msg)
                elif self.mission_active_rover == 2 and not self.r2_broken:
                    self.r2_target_pub.publish(target_msg)
        
        # Gestione Rientro Base (Abort)
        if self.mission_aborted and dist < 0.5 and self.target_x == 0.0:
             if self.target_z != 0.0:
                 self.get_logger().info("BASE REACHED. LANDING.")
                 self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                 self.target_z = 0.0

        self.offboard_setpoint_counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()
