import rclpy  
from rclpy.node import Node  
from rclpy.executors import MultiThreadedExecutor 
from geometry_msgs.msg import Twist, Point  # Twist per comandi di velocità, Point per target
from nav_msgs.msg import Odometry  # Per ricevere la posizione del robot
from sensor_msgs.msg import LaserScan  # Per ricevere dati del LiDAR
from std_msgs.msg import String  
from std_srvs.srv import SetBool  # Servizio booleano per gestire health
from tf_transformations import euler_from_quaternion  # Conversione da quaternion a angoli Eulero
import math  
import threading  

# Stati della macchina a stati del rover
STATE_IDLE = 0  # In attesa
STATE_GO_TO_GOAL = 1  # Sta andando verso il target
STATE_BACKUP = 2  # Retromarcia di emergenza
STATE_AVOID_TURN = 3  # Rotazione per evitare ostacolo
STATE_AVOID_MOVE = 4  # Avanzamento dopo la rotazione
STATE_TARGET_REACHED = 5  # Arrivato al target
STATE_BROKEN = 99  # Stato guasto

class RoverController(Node):  # Definisce la classe del controller del Rover
    def __init__(self, robot_name_input):  
        super().__init__(f'{robot_name_input}_controller') 
        self.robot_name = robot_name_input  
        base_topic = f'/{self.robot_name}'  # Base dei topic

        # Sottoscrizioni ai topic
        self.target_sub = self.create_subscription(Point, f'{base_topic}/target', self.target_callback, 10)  # Target input
        self.odom_sub = self.create_subscription(Odometry, f'{base_topic}/odom', self.odom_callback, 10)  # Odometry
        self.scan_sub = self.create_subscription(LaserScan, f'{base_topic}/scan', self.scan_callback, 10)  # LiDAR

        # Publisher dei comandi e dello stato
        self.cmd_vel_pub = self.create_publisher(Twist, f'{base_topic}/cmd_vel', 10)  # Velocità
        self.status_pub = self.create_publisher(String, f'{base_topic}/status', 10)  # Stato del rover
        self.health_srv = self.create_service(SetBool, f'{base_topic}/set_health', self.set_health_callback)  # Servizio per lo stato di salute

        # Posizione e orientamento attuali del robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  
        self.target_x = None  
        self.target_y = None  
        self.is_healthy = True  
        self.last_status_published = "" 

        # Sensori LiDAR - valori minimi nelle direzioni
        self.min_front = 10.0
        self.min_left = 10.0
        self.min_right = 10.0
        self.too_close = False  # Troppo vicino a un ostacolo

        # Parametri di tuning
        self.trigger_dist = 0.65  
        self.side_safe_dist = 1.0  
        self.critical_dist = 0.35  
        self.max_speed = 1.5  

        # Stato iniziale del rover
        self.current_state = STATE_IDLE
        self.maneuver_counter = 0  # Contatore di manovra
        self.log_counter = 0  # Contatore per pubblicazione stato

        # Durata delle manovre
        self.backup_steps = 10  # Passi in retromarcia
        self.turn_steps = 5  # Passi in rotazione
        self.move_steps = 15  # Passi avanzamento dopo turn

        self.timer = self.create_timer(0.1, self.control_loop)  # Loop di controllo (10 Hz)

        msg = String()
        msg.data = "OK"  
        self.status_pub.publish(msg)
        self.get_logger().info(f">>> {self.robot_name} READY!.")  

    def set_health_callback(self, request, response):  # Servizio health
        if request.data:  
            if not self.is_healthy:  
                self.is_healthy = True
                self.current_state = STATE_IDLE  # Torna idle
                self.target_x = None
                self.target_y = None
                self.get_logger().info(">>> REPAIRED.")
        else:  # Richiesta di rompere il rover
            if self.is_healthy:
                self.is_healthy = False
                self.current_state = STATE_BROKEN
                self.get_logger().warn("!!! BROKEN !!!")
        response.success = True  
        return response

    def target_callback(self, msg):  # Callback del target
        if not self.is_healthy: return  # Se rotto ignora target
        if self.target_x is not None:  # Target già impostato
            if abs(msg.x - self.target_x) < 0.05 and abs(msg.y - self.target_y) < 0.05: return
        self.get_logger().info(f">>> TARGET: {msg.x:.1f}, {msg.y:.1f}")  # Log nuovo target
        self.target_x = msg.x
        self.target_y = msg.y
        self.current_state = STATE_GO_TO_GOAL  # Passa allo stato di movimento
        self.maneuver_counter = 0

    def odom_callback(self, msg):  # Callback odometria
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation  # Quaternion orientamento
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])  # Conversione Eulero
        self.yaw = theta  # Salva yaw

    def scan_callback(self, msg):  # Callback LiDAR
        if self.current_state == STATE_TARGET_REACHED: return
        ranges = msg.ranges  # Lista distanze LiDAR
        if len(ranges) == 0: return

        def get_min(start, end):  # Trova distanza minima in un settore
            m = 100.0
            start = max(0, int(start))
            end = min(len(ranges), int(end))
            for i in range(start, end):
                r = ranges[i]
                # Filtriamo 'inf' e valori troppo piccoli (rumore)
                if not math.isinf(r) and r > 0.05 and r < m:
                    m = r
            return m

        # Il Lidar ha 360 campioni. Indice 0 = destra, 180 = centro, 360 = sinistra
        
        center = len(ranges) / 2  # Angolo zero del LiDAR
        width = 30  # +/- 30° frontali

        self.min_front = get_min(center - width, center + width)  # Ostacoli frontali

        # Settori laterali (repulsione)
        self.min_right = get_min(center - 100, center - 45)
        self.min_left  = get_min(center + 45, center + 100)

        self.too_close = (self.min_front < self.critical_dist)  # Emergenza

    def normalize_angle(self, angle):  # Normalizza angolo [-pi, pi] (per errori di heading)
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def control_loop(self):  # Loop di controllo
        # 1. Pubblicazione Stato (Heartbeat)
        # Lo facciamo ogni 20 cicli (0.5Hz) per non saturare la rete
        self.log_counter += 1
        self.log_counter += 1
        if self.log_counter >= 20:  # Ogni 2 secondi
            msg_status = String()
            msg_status.data = "OK" if self.is_healthy else "BROKEN"
            self.status_pub.publish(msg_status)
            self.log_counter = 0

        msg = Twist()  # Messaggio di velocità

        if self.current_state == STATE_BROKEN:  # Se rotto sta fermo
            self.cmd_vel_pub.publish(msg)
            return
        if self.current_state == STATE_TARGET_REACHED:  # Se arrivato fermo
            self.cmd_vel_pub.publish(msg)
            return
        if self.target_x is None: return  # Se nessun target non muovere

        # Calcolo direzione verso il target
        dist_x = self.target_x - self.x
        dist_y = self.target_y - self.y
        dist_goal = math.sqrt(dist_x**2 + dist_y**2)  # Distanza target
        goal_theta = math.atan2(dist_y, dist_x)  # Angolo verso target
        heading_err = self.normalize_angle(goal_theta - self.yaw)  # Errore di heading

        # EMERGENZA: ostacolo troppo vicino
        if self.too_close and self.current_state != STATE_BACKUP:
            self.current_state = STATE_BACKUP
            self.maneuver_counter = 0

        # MACCHINA A STATI
        if self.current_state == STATE_GO_TO_GOAL:  # Vai al goal
            if dist_goal < 0.2:  # Arrivato
                self.current_state = STATE_TARGET_REACHED
                self.get_logger().info("ARRIVED.")

            elif self.min_front < self.trigger_dist:  # Ostacolo rilevato
                self.current_state = STATE_AVOID_TURN
                self.maneuver_counter = 0

            else:
                # --- Logica di guida con repulsione laterale ---
                base_angular = 1.5 * heading_err  # Rotazione base verso target

                wall_push = 0.0  # Effetto muri laterali

                if self.min_left < self.side_safe_dist:  # Muro sinistro
                    push = (self.side_safe_dist - self.min_left)
                    wall_push -= 2.5 * push

                if self.min_right < self.side_safe_dist:  # Muro destro
                    push = (self.side_safe_dist - self.min_right)
                    wall_push += 2.5 * push

                msg.angular.z = base_angular + wall_push  # Rotazione totale
                msg.angular.z = max(min(msg.angular.z, 2.0), -2.0)  # Limitazione

                # Velocità lineare adattiva
                turn_severity = abs(msg.angular.z)  # Quanto gira
                if turn_severity > 1.0:
                    msg.linear.x = 0.4  # Rallenta
                else:
                    msg.linear.x = self.max_speed  # Vai veloce
                    msg.linear.x *= min(1.0, self.min_front / 2.0)  # Rallenta vicino a ostacoli
                    msg.linear.x = max(0.2, msg.linear.x)

        elif self.current_state == STATE_BACKUP:  # Retromarcia
            msg.linear.x = -0.3
            self.maneuver_counter += 1
            if self.maneuver_counter >= self.backup_steps:
                self.current_state = STATE_AVOID_TURN
                self.maneuver_counter = 0

        elif self.current_state == STATE_AVOID_TURN:  # Rotazione evitamento
            direction = 1.0 if self.min_left > self.min_right else -1.0  # Gira verso lato libero
            msg.angular.z = 0.8 * direction

            self.maneuver_counter += 1
            if self.maneuver_counter > self.turn_steps and self.min_front > 1.5:  # Spazio davanti
                self.current_state = STATE_AVOID_MOVE
                self.maneuver_counter = 0

        elif self.current_state == STATE_AVOID_MOVE:  # Avanza
            msg.linear.x = 0.5 
            msg.angular.z = 0.0 # Vai dritto
            
            self.maneuver_counter += 1
            # Torna alla guida normale se tempo scaduto o se incontra nuovo ostacolo
            if self.maneuver_counter >= self.move_steps or self.min_front < 1.0:
                self.current_state = STATE_GO_TO_GOAL
                self.maneuver_counter = 0

        # Pubblica il comando calcolato ai motori
        self.cmd_vel_pub.publish(msg)

# --- ENTRY POINT DEL PROGRAMMA ---
def main(args=None):
    rclpy.init(args=args) # Inizializza ROS2
    # Crea il nodo
    controller = RoverController('rover_1')
    
    try:
        rclpy.spin(controller) # Mantiene il nodo attivo
    except KeyboardInterrupt:
        pass # Gestisce CTRL+C
    finally:
        # Spegni i motori prima di chiudere
        stop_msg = Twist()
        controller.cmd_vel_pub.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()
