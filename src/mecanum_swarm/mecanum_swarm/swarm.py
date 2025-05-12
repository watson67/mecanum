import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]
GLOBAL_FRAME = "mocap"

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        self.pose_data = {}
        self.last_cmd_vel = Twist()
        self.last_barycenter = None
        self.cmd_vel_publishers = {}

        self.formation_offsets = None
        self.barycenter_yaw = 0.0
        self.offsets_initialized = False
        self.init_time = None
        self.formation_yaw_offset = None

        # contrôle via /master et /formation
        self.active = False
        self.formation_recorded = False

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for name in ROBOT_NAMES:
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel", 10
            )

        self.create_subscription(
            Twist,
            "/Swarm/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Souscription au topic /master (active/désactive le contrôle)
        self.create_subscription(
            Int32,
            "/master",
            self.master_callback,
            10
        )

        # Souscription au topic /formation (déclenche l'enregistrement de la formation)
        self.create_subscription(
            Int32,
            "/formation",
            self.formation_callback,
            10
        )

        self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Variables pour le contrôle de la rotation
        self.current_cmd_vel = Twist()  # Commande actuelle (pas forcément la dernière reçue)
        self.cmd_vel_timeout = 0.5  # Temps en secondes après lequel on considère qu'il n'y a plus de commande
        self.last_cmd_time = None

        # Variables pour le contrôle de la stabilité
        self.robots_initial_poses = None  # Pour stocker les positions initiales exactes des robots
        self.stabilization_phase = False  # Indique si nous sommes dans la phase de stabilisation
        self.stabilization_duration = 2.0  # Durée de la phase de stabilisation en secondes
        self.stabilization_start_time = None

    def master_callback(self, msg):
        was_active = self.active
        self.active = (msg.data == 1)
        
        if self.active and not was_active:
            self.get_logger().info("Contrôle actif - Démarrage de la phase de stabilisation")
            # Réinitialiser les commandes de vitesse pour éviter les mouvements non désirés
            self.current_cmd_vel = Twist()
            self.last_cmd_vel = Twist()
            
            # Activer la phase de stabilisation
            self.stabilization_phase = True
            self.stabilization_start_time = self.get_clock().now()
            
            # Capturer les positions initiales exactes des robots
            self.robots_initial_poses = {}
            for name in ROBOT_NAMES:
                pose = self.get_robot_pose(name)
                if pose is not None:
                    self.robots_initial_poses[name] = pose
        
        elif not self.active and was_active:
            self.get_logger().info("Contrôle désactivé")
            self.stabilization_phase = False
            self.robots_initial_poses = None

    def formation_callback(self, msg):
        # Si msg.data == 1, afficher la position et les distances une seule fois
        if msg.data == 1 and not self.formation_recorded:
            self.display_positions_and_distances()
            self.record_formation()
            self.formation_recorded = True
        elif msg.data == 0:
            self.formation_recorded = False
            self.offsets_initialized = False
            self.formation_offsets = None
            self.formation_yaw_offset = None

    def display_positions_and_distances(self):
        # Récupère et affiche la position de chaque robot et les distances mutuelles
        poses = []
        for name in ROBOT_NAMES:
            pose = self.get_robot_pose(name)
            if pose is None:
                self.get_logger().warn(f"Impossible d'obtenir la pose de {name} pour affichage formation.")
                return
            poses.append((name, pose))
        # Affichage des positions
        self.get_logger().info("Positions des robots pour la formation :")
        for name, (x, y, q) in poses:
            self.get_logger().info(f"{name}: x={x:.3f}, y={y:.3f}")
        # Affichage des distances mutuelles
        self.get_logger().info("Distances entre robots :")
        for i in range(len(poses)):
            for j in range(i+1, len(poses)):
                name1, (x1, y1, _) = poses[i]
                name2, (x2, y2, _) = poses[j]
                dist = math.hypot(x2 - x1, y2 - y1)
                self.get_logger().info(f"{name1} <-> {name2}: {dist:.3f} m")

    def get_robot_pose(self, robot_name):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, f"{robot_name}/base_link", rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            return x, y, q
        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed for {robot_name}: {e}")
            return None

    def record_formation(self):
        poses = []
        for name in ROBOT_NAMES:
            pose = self.get_robot_pose(name)
            if pose is None:
                return
            poses.append(pose)
        x = round(sum(p[0] for p in poses) / len(poses), 6)
        y = round(sum(p[1] for p in poses) / len(poses), 6)
        self.formation_offsets = {}
        for idx, name in enumerate(ROBOT_NAMES):
            rx, ry, _ = poses[idx]
            ox = rx - x
            oy = ry - y
            self.formation_offsets[name] = (ox, oy)
        
        # Calculer l'orientation moyenne des robots pour établir l'orientation initiale
        yaw_sum = 0.0
        for pose in poses:
            q = pose[2]
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            yaw_sum += yaw
        
        avg_yaw = yaw_sum / len(poses)
        self.formation_yaw_offset = avg_yaw
        self.barycenter_yaw = avg_yaw  # Initialiser la direction du barycentre avec la moyenne
        self.offsets_initialized = True
        self.get_logger().info(f"Formation enregistrée: barycentre à ({x:.2f}, {y:.2f}), orientation: {self.barycenter_yaw:.2f}")

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().debug(f"Reçu cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def control_loop(self):
        # Ne rien faire si non actif
        if not self.active:
            return

        # Vérifier si nous sommes en phase de stabilisation
        if self.stabilization_phase:
            current_time = self.get_clock().now()
            if self.stabilization_start_time is not None:
                elapsed = (current_time - self.stabilization_start_time).nanoseconds / 1e9
                if elapsed >= self.stabilization_duration:
                    self.stabilization_phase = False
                    self.get_logger().info("Phase de stabilisation terminée - Contrôle normal activé")
                else:
                    # Pendant la phase de stabilisation, forcer les robots à rester sur leurs positions initiales
                    self._apply_stabilization_control()
                    return

        # Vérifier si nous avons reçu des commandes récemment
        current_time = self.get_clock().now()
        if self.last_cmd_time is not None:
            dt_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
            cmd_active = dt_since_cmd < self.cmd_vel_timeout
        else:
            cmd_active = False
        
        # Si aucune commande récente, utiliser une commande nulle
        if cmd_active:
            self.current_cmd_vel = self.last_cmd_vel
        else:
            # Réduire progressivement la vitesse si aucune commande n'est reçue
            decay_factor = 0.8  # Facteur de décroissance
            self.current_cmd_vel.linear.x *= decay_factor
            self.current_cmd_vel.linear.y *= decay_factor
            self.current_cmd_vel.angular.z *= decay_factor
            
            # Mettre à zéro les très petites valeurs
            if abs(self.current_cmd_vel.linear.x) < 0.01: self.current_cmd_vel.linear.x = 0.0
            if abs(self.current_cmd_vel.linear.y) < 0.01: self.current_cmd_vel.linear.y = 0.0
            if abs(self.current_cmd_vel.angular.z) < 0.01: self.current_cmd_vel.angular.z = 0.0

        poses = []
        for name in ROBOT_NAMES:
            pose = self.get_robot_pose(name)
            if pose is None:
                return
            poses.append(pose)
        x = sum(p[0] for p in poses) / len(poses)
        y = sum(p[1] for p in poses) / len(poses)
        self.last_barycenter = (x, y)

        # Enregistrer la formation uniquement si demandé via /formation
        if not self.offsets_initialized and self.formation_recorded:
            self.record_formation()

        # Ne rien faire tant que la formation n'est pas initialisée
        if not self.offsets_initialized:
            return

        # Mise à jour de l'orientation du barycentre en fonction de la commande angulaire
        dt = 0.05
        vx = self.current_cmd_vel.linear.x
        vy = self.current_cmd_vel.linear.y
        wz = self.current_cmd_vel.angular.z
        
        # Ne mettre à jour l'orientation que si une commande angulaire significative est reçue
        if abs(wz) > 0.01:
            self.barycenter_yaw += wz * dt
        
        bx, by = self.last_barycenter

        # Si aucune commande de mouvement, maintenir la position actuelle du barycentre
        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(wz) < 0.01:
            bx_target = bx
            by_target = by
        else:
            # Conversion des vitesses dans le repère global
            vx_global = vx * math.cos(self.barycenter_yaw) - vy * math.sin(self.barycenter_yaw)
            vy_global = vx * math.sin(self.barycenter_yaw) + vy * math.cos(self.barycenter_yaw)
            
            # Mise à jour de la position cible du barycentre
            bx_target = bx + vx_global * dt
            by_target = by + vy_global * dt

        # Log de debug uniquement si on a un mouvement significatif
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().info(f"Barycentre: ({bx:.2f}, {by:.2f}) -> ({bx_target:.2f}, {by_target:.2f}), orientation: {self.barycenter_yaw:.2f}")

        for idx, name in enumerate(ROBOT_NAMES):
            rx, ry, q = poses[idx]
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

            # Offset de formation pour ce robot (dans le repère du barycentre)
            ox, oy = self.formation_offsets[name]
            # Rotation de l'offset selon l'orientation du barycentre actuelle
            oxg = math.cos(self.barycenter_yaw) * ox - math.sin(self.barycenter_yaw) * oy
            oyg = math.sin(self.barycenter_yaw) * ox + math.cos(self.barycenter_yaw) * oy

            # Position cible pour ce robot
            tx = bx_target + oxg
            ty = by_target + oyg

            # Erreur dans le repère global
            ex = tx - rx
            ey = ty - ry

            # Transformation de l'erreur dans le repère local du robot
            ex_local = math.cos(-yaw) * ex - math.sin(-yaw) * ey
            ey_local = math.sin(-yaw) * ex + math.cos(-yaw) * ey

            # Calcul d'erreur angulaire pour l'orientation des robots
            target_yaw = self.barycenter_yaw
            yaw_error = self.normalize_angle(target_yaw - yaw)
            
            cmd = Twist()
            
            # Gain proportionnel adaptatif: plus petit pour des petites erreurs
            # pour éviter les oscillations
            error_magnitude = math.sqrt(ex_local**2 + ey_local**2)
            if error_magnitude < 0.05:  # Si l'erreur est très petite
                Kp = 0.8  # Gain plus faible pour stabiliser
            else:
                Kp = 1.5  # Gain normal pour des mouvements plus importants
                
            cmd.linear.x = Kp * ex_local
            cmd.linear.y = Kp * ey_local
            
            # Contrôle d'orientation avec gain adaptatif selon la situation
            if abs(wz) > 0.01:  # Si une commande angulaire est active
                cmd.angular.z = wz  # Appliquer directement la commande
            else:
                # Gain plus faible pour le maintien de l'orientation quand pas de commande
                # et adaptation selon l'erreur angulaire pour éviter les oscillations
                if abs(yaw_error) < 0.1:  # 5.7 degrés environ
                    K_yaw = 0.3  # Très faible pour éviter les micro-oscillations
                else:
                    K_yaw = 0.5  # Gain modéré pour les corrections plus importantes
                cmd.angular.z = K_yaw * yaw_error
            
            self.cmd_vel_publishers[name].publish(cmd)

    def _apply_stabilization_control(self):
        """
        Applique un contrôle strict pour maintenir les robots sur leurs positions initiales
        pendant la phase de stabilisation.
        """
        if not self.robots_initial_poses:
            return
            
        for name in ROBOT_NAMES:
            if name not in self.robots_initial_poses:
                continue
                
            # Récupérer la position actuelle et la position initiale
            current_pose = self.get_robot_pose(name)
            if current_pose is None:
                continue
                
            initial_pose = self.robots_initial_poses[name]
            rx, ry, _ = current_pose
            target_x, target_y, q_initial = initial_pose
            
            # Orientation actuelle et initiale
            current_yaw = self.quaternion_to_yaw(current_pose[2].x, current_pose[2].y, current_pose[2].z, current_pose[2].w)
            initial_yaw = self.quaternion_to_yaw(q_initial.x, q_initial.y, q_initial.z, q_initial.w)
            
            # Erreur dans le repère global
            ex = target_x - rx
            ey = target_y - ry
            
            # Transformation de l'erreur dans le repère local du robot
            ex_local = math.cos(-current_yaw) * ex - math.sin(-current_yaw) * ey
            ey_local = math.sin(-current_yaw) * ex + math.cos(-current_yaw) * ey
            
            # Erreur d'orientation
            yaw_error = self.normalize_angle(initial_yaw - current_yaw)
            
            # Appliquer un contrôle forte pour stabiliser rapidement
            cmd = Twist()
            Kp_stab = 2.0  # Gain fort pour la stabilisation
            K_yaw_stab = 1.0  # Gain fort pour l'orientation
            
            cmd.linear.x = Kp_stab * ex_local
            cmd.linear.y = Kp_stab * ey_local
            cmd.angular.z = K_yaw_stab * yaw_error
            
            self.cmd_vel_publishers[name].publish(cmd)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        # Conversion quaternion -> yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle):
        # Normalise un angle entre -π et π
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
