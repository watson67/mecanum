#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import math
from threading import Lock
import concurrent.futures  # Import pour le multi-threading
import time  # Pour mesurer les performances

# Noms des robots dans l'essaim
ROBOT_NAMES = ["Aramis", "Athos", "Porthos"]

# TOPICS ROS pour les poses des robots
POSE_TOPICS = [f"/vrpn_mocap/{name}/pose" for name in ROBOT_NAMES]

# TOPICS ROS pour piloter les robots
CMD_VEL_TOPICS = [f"/{name}/cmd_vel" for name in ROBOT_NAMES]

class ConsensusSwarm(Node):
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
        self.velocities = {name: [0.0, 0.0] for name in ROBOT_NAMES}  # Vitesses actuelles
        self.last_received = {name: None for name in ROBOT_NAMES}
        self.message_counts = {name: 0 for name in ROBOT_NAMES}
        self.position_displayed = {name: False for name in ROBOT_NAMES}
        
        # Variables pour le consensus
        self.errors_integral = {name: [0.0, 0.0] for name in ROBOT_NAMES}  # Erreurs intégrales
        self.goal_position = [0.0, 0.0]  # Position cible du barycentre
        self.mutex = Lock()  # Pour protéger l'accès aux positions
        
        # Création des publishers pour les commandes de vitesse
        self.cmd_vel_publishers = {}
        for name, topic in zip(ROBOT_NAMES, CMD_VEL_TOPICS):
            self.cmd_vel_publishers[name] = self.create_publisher(
                Twist,
                topic,
                self.qos_profile2
            )
            self.get_logger().info(f"Créé publisher pour {name} sur {topic}")

        # Paramètres du consensus inspirés du papier 
        self.n_robots = len(ROBOT_NAMES)
        self.d = 0.5                     # Distance désirée entre robots
        self.r = 1.0                     # Rayon d'interaction
        self.Kp = 0.2                    # Gain proportionnel
        self.Ki = 0.05                   # Gain intégral
        self.c_gamma = 0.25              # Gain de navigation
        self.epsilon = 0.1               # Paramètre pour sigma-norm
        self.dt = 0.05                   # Période d'échantillonnage
        self.max_vel = 0.8               # Vitesse maximale (augmentée de 0.5 à 0.8)

        # Paramètres pour bump function
        self.h = 0.2                     # Paramètre h de la fonction bump
        self.a = 1.0                     # Paramètre a pour Phi_alpha
        self.b = 1.0                     # Paramètre b pour Phi_alpha
        
        # Créer un timer pour exécuter l'algorithme de consensus
        self.timer = self.create_timer(self.dt, self.consensus_control)
    
        # Abonnements aux sujets des poses des robots
        for name, topic in zip(ROBOT_NAMES, POSE_TOPICS):
            self.create_subscription(
                PoseStamped,
                topic,
                self.make_pose_callback(name),
                qos_profile=self.qos_profile
            )

        # Abonnement au sujet de commande globale
        self.create_subscription(
                Twist,
                "Swarm/cmd_vel",
                self.cmd_vel_callback,
                qos_profile=self.qos_profile
            )
        
        # Logs lors de l'initialisation
        self.get_logger().info(f"Subscribed to topics: {POSE_TOPICS}")
        
        # Création d'un pool de threads pour le traitement parallèle des robots
        # Utiliser un nom différent pour éviter le conflit avec l'executor ROS
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=len(ROBOT_NAMES))
        
        # Variables pour mesurer les performances
        self.computation_times = []

    def cmd_vel_callback(self, msg):
        """
        Fonction de callback pour traiter les commandes de vitesse globales.
        Cette commande déplace le barycentre de l'essaim.
        """
        # Extraction des vitesses linéaires et angulaires
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        
        # Mise à jour de la position cible du barycentre en fonction des commandes reçues
        with self.mutex:
            # Calculer la position moyenne actuelle
            valid_positions = [pos for pos in self.positions.values() if pos is not None]
            if valid_positions:
                barycenter = np.mean(valid_positions, axis=0)
                # Mettre à jour la position cible
                self.goal_position[0] = barycenter[0] + linear_x
                self.goal_position[1] = barycenter[1] + linear_y
                self.get_logger().info(f"Nouvelle position cible du barycentre: {self.goal_position}")
        
    # Fonction génératrice pour créer des callbacks spécifiques à chaque robot
    def make_pose_callback(self, name):
        def callback(msg):
            # Vérification du type de message reçu pour éviter les erreurs
            if not isinstance(msg, PoseStamped):
                self.get_logger().error(f"Type de message invalide reçu pour {name}: {type(msg)}")
                return
            
            # Comptage des messages pour des fins de diagnostic
            self.message_counts[name] += 1
            
            # Extraction des coordonnées de position du robot
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # Protection des données partagées avec un mutex pour éviter les conditions de course
            with self.mutex:
                self.positions[name] = np.array([x, y])
            
            # Enregistrement du moment de la dernière réception pour détecter les pertes de communication
            self.last_received[name] = self.get_clock().now()
            
            # Affichage des positions initiales pour faciliter le débogage
            if not self.position_displayed[name]:
                self.get_logger().info(f"Position initiale pour {name}: x={x:.3f}, y={y:.3f}")
                self.position_displayed[name] = True
                
                # Initialisation de la position cible une fois que tous les robots sont localisés
                if all(self.position_displayed.values()):
                    self.get_logger().info("Positions initiales reçues pour tous les robots.")
                    # La position cible est initialisée au barycentre actuel pour éviter des mouvements brusques
                    valid_positions = [pos for pos in self.positions.values() if pos is not None]
                    if valid_positions:
                        barycenter = np.mean(valid_positions, axis=0)
                        self.goal_position = barycenter.tolist()
                        
        return callback
    
    def sigma_norm(self, z):
        """
        Calcule la sigma-norm comme défini dans l'algorithme de consensus.
        Cette norme permet d'éviter les singularités dans le calcul des distances
        et aide à stabiliser les comportements lorsque les robots sont proches.
        """
        norm_z = np.linalg.norm(z)
        return (1/self.epsilon) * (np.sqrt(1 + self.epsilon * norm_z**2) - 1)
    
    def sigma_gradient(self, z):
        """
        Calcule le gradient de la sigma-norm.
        Ce gradient est utilisé pour déterminer la direction des forces d'interaction
        entre les robots de l'essaim.
        """
        norm_z = np.linalg.norm(z)
        return z / np.sqrt(1 + self.epsilon * norm_z**2)
    
    def bump_function(self, s):
        """
        Fonction bump (ou fonction de lissage) définie dans l'algorithme de consensus.
        Elle permet de limiter l'influence des robots en fonction de leur distance:
        - Influence maximale (1.0) quand s < h
        - Décroissance progressive entre h et 1
        - Aucune influence (0.0) quand s > 1
        Cette fonction est essentielle pour définir le rayon d'interaction entre robots.
        """
        if s >= 0 and s < self.h:
            return 1.0
        elif s >= self.h and s <= 1:
            return 0.5 * (1 + np.cos(np.pi * (s - self.h) / (1 - self.h)))
        else:
            return 0.0
    
    def phi_alpha(self, s):
        """Fonction Phi_alpha pour le contrôle de formation"""
        e = abs(self.a - self.b) / np.sqrt(4 * self.a * self.b)
        sigma1 = lambda z: z / np.sqrt(1 + z**2)
        return 0.5 * self.bump_function(s / self.sigma_norm(np.array([self.r, 0]))) * \
               ((self.a + self.b) * sigma1(s - self.sigma_norm(np.array([self.d, 0])) + e) + (self.a - self.b))
    
    def process_robot(self, robot_name, index):
        """
        Traite un robot spécifique et calcule sa commande de vitesse.
        Cette fonction est conçue pour être exécutée dans un thread dédié.
        
        Args:
            robot_name: Nom du robot à traiter
            index: Index du robot dans la liste ROBOT_NAMES
        
        Returns:
            tuple: (robot_name, commande Twist calculée)
        """
        # Position actuelle du robot i
        pi = self.positions[robot_name]
        
        # Terme de formation (u_alpha)
        u_alpha = np.array([0.0, 0.0])
        
        for j, neighbor_name in enumerate(ROBOT_NAMES):
            if index == j:  # Ignorer le robot lui-même
                continue
            
            # Position du voisin j
            pj = self.positions[neighbor_name]
            
            # Différence de position
            diff = pj - pi
            
            # Calcul de la norme et du gradient
            norm_diff = self.sigma_norm(diff)
            nij = self.sigma_gradient(diff)
            
            # Adjacency matrix (simplified for full connectivity)
            aij = self.bump_function(norm_diff / self.sigma_norm(np.array([self.r, 0])))
            
            # Consensus term
            u_alpha += self.Kp * self.phi_alpha(norm_diff) * nij
            
            # Intégration pour le terme intégral
            self.errors_integral[robot_name] += self.Ki * self.phi_alpha(norm_diff) * nij * self.dt
        
        # Ajouter le terme intégral
        u_alpha += self.errors_integral[robot_name]
        
        # Terme de navigation (u_gamma)
        # Calculer le barycentre actuel
        barycenter = np.mean([self.positions[name] for name in ROBOT_NAMES], axis=0)
        
        # Calculer l'erreur pour atteindre la position cible
        error_to_goal = np.array(self.goal_position) - barycenter
        
        # Distribuer l'erreur à chaque robot
        u_gamma = -self.c_gamma * (pi - (np.array(self.positions[robot_name]) + error_to_goal))
        
        # Somme des termes de contrôle
        u_total = u_alpha + u_gamma
        
        # Limiter la vitesse
        norm_u = np.linalg.norm(u_total)
        if norm_u > self.max_vel:
            u_total = u_total * (self.max_vel / norm_u)
        
        # Créer la commande de vitesse
        cmd = Twist()
        cmd.linear.x = float(u_total[0])
        cmd.linear.y = float(u_total[1])
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        return (robot_name, cmd)
    
    def consensus_control(self):
        """
        Exécute l'algorithme de consensus pour maintenir la formation
        en utilisant un pool de threads pour traiter chaque robot en parallèle
        """
        start_time = time.time()
        
        with self.mutex:
            # Vérifier si toutes les positions sont disponibles
            if not all(pos is not None for pos in self.positions.values()):
                return
            
            # Liste des futures pour collecter les résultats
            futures = []
            
            # Soumettre les tâches de traitement de robot au pool de threads
            for i, robot_name in enumerate(ROBOT_NAMES):
                future = self.thread_pool.submit(self.process_robot, robot_name, i)
                futures.append(future)
            
            # Collecter les résultats et publier les commandes
            for future in concurrent.futures.as_completed(futures):
                try:
                    robot_name, cmd = future.result()
                    self.cmd_vel_publishers[robot_name].publish(cmd)
                    self.get_logger().debug(f"Robot {robot_name}: vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f}")
                except Exception as e:
                    self.get_logger().error(f"Erreur lors du traitement en parallèle: {str(e)}")
        
        # Mesurer le temps d'exécution
        execution_time = time.time() - start_time
        self.computation_times.append(execution_time)
        
        # Afficher les statistiques de performance tous les 100 cycles
        if len(self.computation_times) % 100 == 0:
            avg_time = sum(self.computation_times[-100:]) / 100
            self.get_logger().info(f"Temps moyen de calcul par cycle (100 derniers): {avg_time*1000:.2f} ms")

# Point d'entrée principal du script
def main(args=None):
    rclpy.init(args=args)
    node = ConsensusSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()