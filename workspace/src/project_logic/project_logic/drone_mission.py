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

        # --- QoS Settings ---
        qos_px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_status = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Publishers ---
        # PX4 Control
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_px4)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_px4)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_px4)
        
        # Rover Control
        self.r1_target_pub = self.create_publisher(Point, '/rover_1/target', 10)
        self.r2_target_pub = self.create_publisher(Point, '/rover_emergency/target', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/rover_1/status', self.r1_status_cb, qos_status)
        self.create_subscription(String, '/rover_emergency/status', self.r2_status_cb, qos_status)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_px4)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odom_callback, qos_px4)

        # --- Stato Interno ---
        self.current_position = [0.0, 0.0, 0.0] # [North, East, Down]
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0 
        
        self.offboard_setpoint_counter_ = 0 
        self.tick_counter = 0               
        
        # Logica Missione / Flotta
        self.r1_broken = False
        self.r2_broken = False
        self.mission_aborted = False 
        self.mission_active_rover = 1 
        self.home_pos_r2 = Point(x=2.0, y=-3.0, z=0.0)
        
        # Logica Controllo
        self.drone_arrived = False   
        self.mission_started = False  # Interruttore principale per inviare ai rover
        self.manual_mode = False      # Se True, non inviamo setpoint al drone, ma gestiamo i rover

        # Timer Antirimbalzo Status (Debounce)
        self.last_r1_change = 0.0
        self.last_r2_change = 0.0
        self.debounce = 2.0

        # --- Avvio Loop ---
        # Timer principale a 10 Hz (0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Thread input utente (Daemon così muore se muore il nodo)
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True 
        self.input_thread.start()

    # --- Callbacks PX4 ---
    def vehicle_status_callback(self, msg): pass
    
    def vehicle_odom_callback(self, msg): 
        # PX4 coordinate frame: NED (North, East, Down)
        # North = Y per noi, East = X per noi
        self.current_position = msg.position

    def get_time(self): return self.get_clock().now().nanoseconds / 1e9

    # --- INPUT LOOP (Gestione Terminale) ---
    def user_input_loop(self):
        # Attesa iniziale per pulire il buffer di stampa
        time.sleep(1.0)
        print("\n=================================================")
        print("   DRONE MISSION CONTROL CENTER")
        print("=================================================")
        print("Select Mode:")
        print(" [1] AUTO MODE   (Drone flies via script coordinates)")
        print(" [2] MANUAL MODE (You fly with Joystick, script sends rover targets)")
        
        # Selezione Modalità
        while True:
            try:
                mode_sel = input(">>> Select (1/2): ").strip()
                if mode_sel == '2':
                    self.manual_mode = True
                    print("\n>>> MANUAL MODE SELECTED. Use QGroundControl to fly.")
                    print("Commands: 'start' (send rovers to drone pos), 'stop' (pause rovers)")
                    break
                elif mode_sel == '1':
                    self.manual_mode = False
                    print("\n>>> AUTO MODE SELECTED.")
                    break
            except ValueError: pass

        # Loop Comandi continuo
        while rclpy.ok():
            try:
                # ----------------- MANUAL MODE INPUT -----------------
                if self.manual_mode:
                    cmd = input("\n[MANUAL CMD] (start/stop): ").strip().lower()
                    
                    if cmd == 'start':
                        # 1. Cattura posizione attuale drone
                        self.target_x = float(self.current_position[1]) # East -> X
                        self.target_y = float(self.current_position[0]) # North -> Y
                        
                        # 2. Attiva l'invio nel timer
                        self.mission_started = True
                        
                        print(f">>> TARGET LOCKED: ({self.target_x:.2f}, {self.target_y:.2f})")
                        print(">>> ROVERS ARE MOVING...")
                    
                    elif cmd == 'stop':
                        self.mission_started = False
                        print(">>> MISSION STOPPED. Rovers holding position.")
                    
                    else:
                        print(f"Unknown command: {cmd}")

                # ----------------- AUTO MODE INPUT -----------------
                else: 
                    # Logica Auto
                    if not self.mission_started:
                        print("\n[AUTO] Insert target coordinates:")
                        try:
                            in_x = input("Target X: ")
                            in_y = input("Target Y: ")
                            self.target_x = float(in_x)
                            self.target_y = float(in_y)
                        except ValueError:
                            print("Invalid number.")
                            continue
                        
                        self.drone_arrived = False
                        print(f">>> Drone flying to ({self.target_x}, {self.target_y})...")
                        
                        # Attesa "passiva" finché il timer non segnala l'arrivo
                        while not self.drone_arrived and rclpy.ok():
                            time.sleep(0.5)

                        print(f"\n>>> DRONE ARRIVED ({self.target_x}, {self.target_y}).")
                        cmd = input("[COMMAND] Start rover mission? (start/no): ").strip().lower()
                        
                        if cmd == 'start':
                            self.mission_started = True
                            print(">>> MISSION STARTED.")
                            # Attesa reset
                            input("Press ENTER to reset/stop mission...")
                            self.mission_started = False
                            print(">>> MISSION STOPPED.")
                        else:
                            print(">>> Canceled.")

            except EOFError:
                # Se l'input crasha aspettiamo
                time.sleep(1)
            except Exception as e:
                print(f"Input error: {e}")
                time.sleep(1)

    # --- LOGICA FDIR (Fault Detection) ---
    def check_global_failure(self):
        if self.r1_broken and self.r2_broken and not self.mission_aborted:
            self.get_logger().error("\n!!! CRITICAL: BOTH ROVERS BROKEN. RETURN TO BASE !!!")
            self.mission_aborted = True
            self.mission_started = False
            # 
            if not self.manual_mode:
                self.target_x = 0.0
                self.target_y = 0.0

    def r1_status_cb(self, msg):
        if not self.mission_started: return
        now = self.get_time()
        is_broken = (msg.data == "BROKEN")

        if is_broken and not self.r1_broken:
            if (now - self.last_r1_change) > self.debounce:
                self.get_logger().error("!!! ALARM: ROVER 1 BROKEN! EMERGENCY ROVER ACTIVE !!!")
                self.r1_broken = True
                self.last_r1_change = now
                self.check_global_failure()
                # Switch immediato
                if not self.mission_aborted and not self.r2_broken:
                    self.mission_active_rover = 2
                    self.r2_target_pub.publish(Point(x=self.target_x, y=self.target_y, z=0.0))

        elif not is_broken and self.r1_broken:
            if (now - self.last_r1_change) > self.debounce:
                self.get_logger().info(">>> ROVER 1 REPAIRED. EMERGENCY ROVER COMING TO BASE.")
                self.r1_broken = False
                self.last_r1_change = now
                self.mission_active_rover = 1
                self.mission_aborted = False
                # Restore order
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
        msg.position = [y, x, z] # NED swap
        msg.yaw = 0.0 
        self.trajectory_setpoint_publisher_.publish(msg)

    # --- TIMER PRINCIPALE (Heartbeat del sistema) ---
    def timer_callback(self):
        self.tick_counter += 1

        # A) LOGICA VOLO DRONE (Solo se in AUTO)
        if not self.manual_mode:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.target_x, self.target_y, self.target_z)
            
            if self.offboard_setpoint_counter_ == 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            # Check arrivo
            curr_x = self.current_position[1]
            curr_y = self.current_position[0]
            dist = math.sqrt((self.target_x - curr_x)**2 + (self.target_y - curr_y)**2)

            if dist < 0.5 and not self.drone_arrived and self.offboard_setpoint_counter_ > 200:
                self.drone_arrived = True 

            # Rientro Base abort
            if self.mission_aborted and dist < 0.5 and self.target_x == 0.0:
                 if self.target_z != 0.0:
                     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                     self.target_z = 0.0
            
            self.offboard_setpoint_counter_ += 1

        #LOGICA ROVER (sia in MANUAL che in AUTO)
        # Se la missione è attiva, inviamo i target continuamente
        if self.mission_started and not self.mission_aborted:
            # Invio a circa 1Hz (ogni 10 tick da 0.1s)
            if self.tick_counter % 10 == 0:
                tx = float(self.target_x)
                ty = float(self.target_y)
                target_msg = Point(x=tx, y=ty, z=0.0)
                
                if self.mission_active_rover == 1 and not self.r1_broken:
                    self.r1_target_pub.publish(target_msg)
                elif self.mission_active_rover == 2 and not self.r2_broken:
                    self.r2_target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e: print(e)
    finally: node.destroy_node(); rclpy.shutdown()
