#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Noms des robots dans l'essaim
ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

# TOPICS ROS pour les poses des robots
POSE_TOPICS = [f"/vrpn_mocap/{name}/pose" for name in ROBOT_NAMES]

# TOPICS ROS pour piloter les robots
CMD_VEL_TOPICS = [f"/{name}/cmd_vel" for name in ROBOT_NAMES]

class PoseMonitor(Node):
    def __init__(self):
        super().__init__('consensus_swarm')

        # Configuration QoS
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Configuration QoS pour cmd_vel
        self.qos_profile2 = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Stockage des données des robots
        self.positions = {name: None for name in ROBOT_NAMES}
        self.last_received = {name: None for name in ROBOT_NAMES}
        self.message_counts = {name: 0 for name in ROBOT_NAMES}
        self.position_displayed = {name: False for name in ROBOT_NAMES}
        
        # Création des publishers pour les commandes de vitesse
        self.cmd_vel_publishers = {}
        for name, topic in zip(ROBOT_NAMES, CMD_VEL_TOPICS):
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist,
                topic,
                self.qos_profile2
            )
            self.get_logger().info(f"Créé publisher pour {name} sur {topic}")

        # Paramètres du consensus 
        self.n_robots = 3
        self.d = 2                      # Distance désirée entre robots
        self.c = 5                      # Rayon d'interaction
        self.Kp = 0.2                   # Gain proportionnel
        self.Ki = 0.05                  # Gain intégral
        self.Kd = 0                     # Gain dérivatif
        self.c_beta = 0.3               # Gain d'évitement d'obstacles
        self.c_gamma = 0.25             # Gain de navigation
        self.epsilon = 0.1              # Paramètre pour sigma-norm
        self.detection_range = 3        # Rayon de détection d'obstacles

    
        # Abonnements aux sujets des poses des robots (version optimisée)
        for name, topic in zip(ROBOT_NAMES, POSE_TOPICS):
            self.create_subscription(
                PoseStamped,
                topic,
                self.make_pose_callback(name),
                qos_profile=self.qos_profile
            )

        self.create_subscription(
                Twist,
                "Swarm/cmd_vel",
                self.cmd_vel_callback,
                qos_profile=self.qos_profile
            )
        
        # Logs lors de l'initialisation
        self.get_logger().info(f"Subscribed to topics: {POSE_TOPICS}")

    def cmd_vel_callback(self, msg):
        """
        Fonction de callback pour traiter les commandes de vitesse.
        
        Args:
            msg (Twist): Message de commande de vitesse contenant les vitesses linéaires et angulaires
        """
        # Extraction des vitesses linéaires et angulaires
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # Journalisation de la commande reçue
        self.get_logger().info(f"Commande de vitesse reçue: linéaire=({linear_x:.2f}, {linear_y:.2f}), angulaire={angular_z:.2f}")
        
        # Ici sera implémentée la logique de l'algorithme de consensus
        # - Calcul des vitesses désirées basé sur les règles de consensus
        # - Prise en compte des positions de tous les robots
        # - Application de la loi de contrôle de consensus
        
        # Pour l'instant, on se contente de print la réception de la commande
        self.get_logger().debug("Traitement de la commande de vitesse avec l'algorithme de consensus")
        
        # Temporaire: envoi de la même commande à tous les robots
        for name, publisher in self.cmd_vel_publishers.items():
            publisher.publish(msg)
            self.get_logger().debug(f"Commande envoyée à {name}")

    # Fonction génératrice pour les callbacks de pose
    def make_pose_callback(self, name):
        def callback(msg):
            if not isinstance(msg, PoseStamped):
                self.get_logger().error(f"Type de message invalide reçu pour {name}: {type(msg)}")
                return
            
            self.message_counts[name] += 1
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.positions[name] = (x, y)
            self.last_received[name] = self.get_clock().now()
            
            # Affiche la position seulement la première fois
            if not self.position_displayed[name]:
                self.get_logger().info(f"Position initiale pour {name}: x={x:.3f}, y={y:.3f}")
                self.position_displayed[name] = True
                
                # Vérifier si on a reçu la position de tous les robots
                if all(self.position_displayed.values()):
                    self.get_logger().info("Positions initiales reçues pour tous les robots.")
        return callback
    
    def compute_pid(self, error_x, error_y, integral_x, integral_y, prev_error_x, prev_error_y):
        
        up_x = self.Kp * error_x
        up_y = self.Kp * error_y

        ud_x = self.Kd * (error_x - prev_error_x) / self.dt
        ud_y = self.Kd * (error_y - prev_error_y) / self.dt

        integral_x += self.Ki * error_x * self.dt
        integral_y += self.Ki * error_y * self.dt

        vx = -(up_x + ud_x + integral_x)
        vy = -(up_y + ud_y + integral_y)

        return vx, vy, integral_x, integral_y, error_x, error_y

    def compute_pid_angle(self, error_angle, integral_angle, prev_error_angle):
        up = self.Kp * error_angle
        ud = self.Kd * (error_angle - prev_error_angle) / self.dt
        integral_angle += self.Ki * error_angle * self.dt

        angular_z = up + ud + integral_angle
        return angular_z, integral_angle, error_angle


# Point d'entrée principal du script
def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
