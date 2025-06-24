#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import math
from threading import Lock
import concurrent.futures
import time

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
        self.velocities = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}  # Vitesses actuelles
        self.prev_positions = {name: None for name in ROBOT_NAMES}  # Positions précédentes pour estimer les vitesses
        self.last_received = {name: None for name in ROBOT_NAMES}
        self.message_counts = {name: 0 for name in ROBOT_NAMES}
        self.position_displayed = {name: False for name in ROBOT_NAMES}
        
        # Variables pour le consensus
        self.errors_integral = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}  # Erreurs intégrales
        self.goal_position = np.array([0.0, 0.0])  # Position cible du barycentre
        self.mutex = Lock()  # Pour protéger l'accès aux positions
        self.initial_formation = {name: None for name in ROBOT_NAMES}  # Stockage de la formation initiale
        
        # Variable pour contrôler l'état de l'algorithme
        self.consensus_enabled = False
        self.consensus_startup_message_displayed = False  # Pour n'afficher le message qu'une fois
        self.get_logger().info("Consensus désactivé. En attente du signal sur le topic /master")
        
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
        # "Consensus-based formation control and obstacle avoidance for nonholonomic multi-robot system"
        self.n_robots = len(ROBOT_NAMES)
        self.d = 0.5                     # Distance désirée entre robots (équation 2)
        self.r = 1.0                     # Rayon d'interaction (équation 1)
        
        # Gains de contrôle (équation 18)
        self.Kp = 0.15                   # Gain proportionnel pour le terme Phi_alpha
        self.Kp_prime = 0.1              # Gain proportionnel pour le terme avec aij
        self.Ki = 0.02                   # Gain intégral
        self.Kd = 0.08                   # Gain différentiel pour le terme avec aij(p)(vj-vi)
        
        self.c_gamma = 0.25              # Gain de navigation (équation 14, terme u_gamma)
        self.epsilon = 0.1               # Paramètre pour sigma-norm (équation 4)
        self.dt = 0.05                   # Période d'échantillonnage
        self.max_vel = 0.8               # Vitesse maximale
        self.min_vel_threshold = 0.05    # Seuil de vitesse minimale

        # Paramètres pour bump function (équation 7)
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
            
        # Abonnement au topic master pour contrôler l'activation du consensus
        self.create_subscription(
            Int32,
            '/master',
            self.master_callback,
            qos_profile=self.qos_profile2
        )
        
        # Logs lors de l'initialisation
        self.get_logger().info(f"Subscribed to topics: {POSE_TOPICS}")
        
        # Création d'un pool de threads pour le traitement parallèle des robots
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=len(ROBOT_NAMES))
        
        # Variables pour mesurer les performances
        self.computation_times = []

    def master_callback(self, msg):
        """
        Fonction callback pour le topic /master qui active ou désactive l'algorithme de consensus
        """
        if msg.data == 1 and not self.consensus_enabled:
            self.consensus_enabled = True
            
            # N'afficher le message de démarrage que la première fois
            if not self.consensus_startup_message_displayed:
                self.get_logger().info("Consensus activé suite à la réception d'un 1 sur le topic /master")
                self.consensus_startup_message_displayed = True
            else:
                self.get_logger().info("Consensus réactivé")
                
        elif msg.data == 0 and self.consensus_enabled:
            self.consensus_enabled = False
            self.consensus_startup_message_displayed = False  # Réinitialiser pour le prochain démarrage
            self.get_logger().info("Consensus désactivé")
            
            # Réinitialiser les intégrales d'erreur pour éviter l'accumulation
            for name in ROBOT_NAMES:
                self.errors_integral[name] = np.array([0.0, 0.0])

    def cmd_vel_callback(self, msg):
        """
        Fonction de callback pour traiter les commandes de vitesse globales.
        Cette commande déplace le barycentre de l'essaim.
        """
        # Extraction des vitesses linéaires
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
                
                # Si la commande est proche de zéro, réinitialiser les intégrales
                if abs(linear_x) < 0.01 and abs(linear_y) < 0.01:
                    for name in ROBOT_NAMES:
                        self.errors_integral[name] = np.array([0.0, 0.0])
                    self.get_logger().info("Commande proche de zéro: réinitialisation des intégrales d'erreur")
        
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
            current_position = np.array([x, y])
            
            # Protection des données partagées avec un mutex pour éviter les conditions de course
            with self.mutex:
                # Sauvegarder la position précédente pour le calcul de vitesse
                if self.positions[name] is not None:
                    self.prev_positions[name] = self.positions[name].copy()
                else:
                    self.prev_positions[name] = current_position.copy()
                
                # Mettre à jour la position actuelle
                self.positions[name] = current_position
                
                # Estimer la vitesse actuelle
                if self.prev_positions[name] is not None:
                    # Calcul de la vitesse comme dérivée de la position
                    self.velocities[name] = (current_position - self.prev_positions[name]) / self.dt
            
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
                        self.goal_position = barycenter.copy()
                        
                        # Initialiser la formation initiale comme référence
                        self.initial_formation = {n: pos.copy() if pos is not None else None 
                                                for n, pos in self.positions.items()}
                        self.get_logger().info("Formation initiale enregistrée comme référence")
                        
        return callback
    
    def sigma_norm(self, z):
        """
        Calcule la sigma-norm comme défini dans l'équation (4) du papier.
        Cette norme permet d'éviter les singularités dans le calcul des distances
        et aide à stabiliser les comportements lorsque les robots sont proches.
        """
        norm_z = np.linalg.norm(z)
        return (1/self.epsilon) * (np.sqrt(1 + self.epsilon * norm_z**2) - 1)
    
    def sigma_gradient(self, z):
        """
        Calcule le gradient de la sigma-norm (équation 5).
        Ce gradient est utilisé pour déterminer la direction des forces d'interaction
        entre les robots de l'essaim.
        """
        norm_z = np.linalg.norm(z)
        return z / np.sqrt(1 + self.epsilon * norm_z**2)
    
    def bump_function(self, s):
        """
        Fonction bump (ou fonction de lissage) définie dans l'équation (7) du papier.
        Elle permet de limiter l'influence des robots en fonction de leur distance:
        - Influence maximale (1.0) quand s < h
        - Décroissance progressive entre h et 1
        - Aucune influence (0.0) quand s > 1
        """
        if s >= 0 and s < self.h:
            return 1.0
        elif s >= self.h and s <= 1:
            return 0.5 * (1 + np.cos(np.pi * (s - self.h) / (1 - self.h)))
        else:
            return 0.0
    
    def phi_alpha(self, s):
        """
        Fonction Phi_alpha pour le contrôle de formation,
        définie en relation avec l'équation (8) du papier.
        """
        e = abs(self.a - self.b) / np.sqrt(4 * self.a * self.b)
        sigma1 = lambda z: z / np.sqrt(1 + z**2)
        
        c_sigma_norm = self.sigma_norm(np.array([self.r, 0]))
        d_sigma_norm = self.sigma_norm(np.array([self.d, 0]))
        
        return 0.5 * self.bump_function(s / c_sigma_norm) * \
               ((self.a + self.b) * sigma1(s - d_sigma_norm + e) + (self.a - self.b))
    
    def process_robot(self, robot_name, index):
        """
        Traite un robot spécifique et calcule sa commande de vitesse selon l'équation (18) du papier.
        Cette fonction est conçue pour être exécutée dans un thread dédié.
        
        Args:
            robot_name: Nom du robot à traiter
            index: Index du robot dans la liste ROBOT_NAMES
        
        Returns:
            tuple: (robot_name, commande Twist calculée)
        """
        # Position et vitesse actuelles du robot i
        pi = self.positions[robot_name]
        vi = self.velocities[robot_name]
        
        # Initialisation des termes de contrôle (équation 18)
        # u_i = u_i^alpha + u_i^gamma
        u_alpha = np.array([0.0, 0.0])  # Terme de formation
        u_gamma = np.array([0.0, 0.0])  # Terme de navigation
        
        # Calcul du terme de formation (u_alpha) basé sur l'équation (18)
        for j, neighbor_name in enumerate(ROBOT_NAMES):
            if index == j:  # Ignorer le robot lui-même
                continue
            
            # Position et vitesse du voisin j
            pj = self.positions[neighbor_name]
            vj = self.velocities[neighbor_name]
            
            # Différence de position et de vitesse (pj - pi) et (vj - vi)
            diff_pos = pj - pi
            diff_vel = vj - vi
            
            # Calcul de la norme et du gradient (équations 4 et 5)
            norm_diff = self.sigma_norm(diff_pos)
            nij = self.sigma_gradient(diff_pos)
            
            # Calcul de la matrice d'adjacence (équation 6)
            aij = self.bump_function(norm_diff / self.sigma_norm(np.array([self.r, 0])))
            
            # Premier terme de l'équation (18): Kp * Phi_alpha * nij
            u_alpha += self.Kp * self.phi_alpha(norm_diff) * nij
            
            # Deuxième terme de l'équation (18): K'p * aij * (pj - pi)
            u_alpha += self.Kp_prime * aij * diff_pos
            
            # Troisième terme de l'équation (18): Kd * aij * (vj - vi)
            u_alpha += self.Kd * aij * diff_vel
            
            # Quatrième terme de l'équation (18): Ki * intégrale de (Phi_alpha * nij)
            # Intégration du terme intégral
            self.errors_integral[robot_name] += self.Ki * self.phi_alpha(norm_diff) * nij * self.dt
        
        # Ajouter le terme intégral
        u_alpha += self.errors_integral[robot_name]
        
        # Terme de navigation (u_gamma) basé sur l'équation (18)
        # Calculer le barycentre actuel
        barycenter = np.mean([self.positions[name] for name in ROBOT_NAMES], axis=0)
        
        # Calculer l'erreur pour atteindre la position cible
        error_to_goal = self.goal_position - barycenter
        
        # Calculer le décalage relatif à la position initiale pour maintenir la formation
        initial_offset = np.zeros(2)
        if all(pos is not None for pos in self.initial_formation.values()):
            initial_barycenter = np.mean([self.initial_formation[name] for name in ROBOT_NAMES], axis=0)
            initial_offset = self.initial_formation[robot_name] - initial_barycenter
        
        # Distribuer l'erreur à chaque robot en préservant la formation initiale
        # Terme u_gamma = -c_gamma1 * (pi - pr) de l'équation (18)
        u_gamma = -self.c_gamma * (pi - (barycenter + error_to_goal + initial_offset))
        
        # Somme des termes de contrôle (u_i = u_i^alpha + u_i^gamma)
        u_total = u_alpha + u_gamma
        
        # Zone morte: si l'erreur est petite, on ne bouge pas
        position_error = np.linalg.norm(self.goal_position - barycenter)
        if position_error < 0.1:  # 10cm de tolérance pour le barycentre
            robot_error = np.linalg.norm(u_total)
            if robot_error < 0.2:  # Si la correction individuelle est aussi petite
                u_total = np.array([0.0, 0.0])
        
        # Limiter la vitesse
        norm_u = np.linalg.norm(u_total)
        if norm_u > self.max_vel:
            u_total = u_total * (self.max_vel / norm_u)
        elif norm_u < self.min_vel_threshold:
            u_total = np.array([0.0, 0.0])
        
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
        # Vérifier si l'algorithme est activé
        if not self.consensus_enabled:
            return
            
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