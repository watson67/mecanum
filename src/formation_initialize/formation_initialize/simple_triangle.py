import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class FormationInitializer(Node):
    def __init__(self):
        super().__init__('formation_initializer')

        self.robot_names = ["Aramis", "Athos", "Porthos"]

        # Topics
        self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
        self.cmd_vel_topics = [f"/{name}/cmd_vel" for name in self.robot_names]
        
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers pour les vitesses
        self.cmd_publishers = []
        for topic in self.cmd_vel_topics:
            pub = self.create_publisher(Twist, topic, 10)
            self.cmd_publishers.append(pub)
            
        # Publisher pour les trajectoires planifiées
        self.planned_trajectory_publisher = self.create_publisher(
            Float32MultiArray, 
            '/formation_initializer/planned_trajectories', 
            10
        )

        # Souscriptions aux poses
        self.positions = {name: None for name in self.robot_names}
        self.orientations = {name: None for name in self.robot_names}  # Pour stocker les orientations (yaw)
        self.velocities = {name: (0.0, 0.0) for name in self.robot_names}
        self.prev_positions = {name: None for name in self.robot_names}
        self.last_update_time = {name: None for name in self.robot_names}
        
        # Variables pour contrôler les phases d'orientation
        self.orientation_phase = {name: "initial" for name in self.robot_names}  # "initial", "moving", "final"
        self.orientation_tolerance = 0.05  # Tolérance angulaire en radians (environ 3 degrés)
        self.target_orientation = 0.0  # Orientation cible désirée (0 radians = orientation selon +x)
        
        for i, topic in enumerate(self.pose_topics):
            self.create_subscription(PoseStamped, topic, 
                                     self.pose_callback_factory(self.robot_names[i]),
                                    qos_profile=self.qos_profile)

        # Positions cibles
        self.targets = {
            "Aramis": (-1, 0),
            "Athos": (0.5, 0.5),
            "Porthos": (0.5, -0.5)
        }

        # Trajectoires planifiées pour chaque robot
        self.trajectories = {name: None for name in self.robot_names}
        self.current_waypoint_index = {name: 0 for name in self.robot_names}
        
        # Paramètres pour la planification de trajectoire
        self.waypoint_tolerance = 0.1   # Distance à laquelle un waypoint est considéré atteint
        self.collision_distance = 0.2   # Augmenté pour faire des détours plus prononcés
        self.waypoint_spacing = 0.4     # Augmenté pour réduire le nombre de waypoints
        self.robot_radius = 0.2        # Rayon physique des robots (20 cm)
        self.safety_margin = 0.05       # Marge de sécurité supplémentaire
        self.has_planned = False       # Indique si les trajectoires ont été planifiées
        
        # Paramètres pour les courbes de Bézier
        self.use_bezier_curves = True   # Activer les courbes de Bézier
        self.bezier_resolution = 20     # Nombre de points sur une courbe de Bézier
        self.bezier_control_scale = 0.7 # Échelle des points de contrôle
        
        # Paramètres de contrôle
        self.normal_speed = 0.8        # Réduit: vitesse normale plus douce
        self.k_p = 0.7                 # Réduit: gain proportionnel moins agressif
        self.k_d = 0.3                 # Augmenté: plus d'amortissement pour réduire les oscillations
        
        # Paramètres pour le lissage des commandes
        self.smoothing_factor = 0.8    # Augmenté pour plus de lissage
        self.previous_commands = {name: (0.0, 0.0) for name in self.robot_names}
        
        # Paramètres pour l'accélération et la décélération progressives
        self.max_acceleration = 0.08    # Réduit pour des accélérations plus progressives
        self.waypoint_slowdown = 0.5    # Augmenté pour une approche plus douce des waypoints
        
        # Autres paramètres de trajectoire
        self.final_approach_distance = 0.5  # Augmenté pour une approche finale plus douce

        # Ajout d'une file de trajectoire pour un meilleur suivi
        self.lookahead_distance = 0.3   # Distance de regard en avant pour le suivi de trajectoire

        # Timer pour la boucle de contrôle
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variable pour suivre l'état de la formation
        self.formation_complete = False
        
        # Paramètres pour la détection et la gestion des blocages
        self.stuck_detection_time = 2.0  # Temps en secondes pour détecter un robot bloqué
        self.stuck_distance_threshold = 0.05  # Si le robot ne s'est pas déplacé de cette distance, il est considéré bloqué
        self.last_progress_positions = {name: None for name in self.robot_names}
        self.last_progress_time = {name: None for name in self.robot_names}
        self.robot_stuck_count = {name: 0 for name in self.robot_names}
        self.trajectory_recalculation_cooldown = {name: 0 for name in self.robot_names}
        self.max_recalculations = 3  # Nombre maximal de recalculations par robot

        # Logging pour le débogage
        self.get_logger().info("Formation initializer started")

    def pose_callback_factory(self, name):
        def callback(msg):
            current_time = self.get_clock().now()
            current_pos = (msg.pose.position.x, msg.pose.position.y)
            
            # Calculer la vitesse si on a une position précédente
            if self.prev_positions[name] is not None and self.last_update_time[name] is not None:
                dt = (current_time - self.last_update_time[name]).nanoseconds / 1e9
                if dt > 0:
                    vx = (current_pos[0] - self.prev_positions[name][0]) / dt
                    vy = (current_pos[1] - self.prev_positions[name][1]) / dt
                    self.velocities[name] = (vx, vy)
            
            # Extraire l'orientation (yaw) à partir du quaternion
            x = msg.pose.orientation.x
            y = msg.pose.orientation.y
            z = msg.pose.orientation.z
            w = msg.pose.orientation.w
            
            # Conversion quaternion vers euler (yaw)
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Normaliser l'angle entre -pi et pi
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
                
            self.orientations[name] = yaw
            self.positions[name] = current_pos
            self.prev_positions[name] = current_pos
            self.last_update_time[name] = current_time
            
        return callback

    def is_formation_complete(self):
        """Vérifie si tous les robots ont atteint leur cible"""
        if any(position is None for position in self.positions.values()):
            return False
            
        for name, position in self.positions.items():
            target = self.targets[name]
            distance = math.hypot(target[0] - position[0], target[1] - position[1])
            if distance > 0.05:  # même tolérance que dans control_loop
                return False
        
        return True

    def plan_trajectories(self):
        """Planifie les trajectoires pour tous les robots en évitant les collisions"""
        self.get_logger().info("Planning trajectories to avoid collisions...")
        
        # Vérifier que toutes les positions initiales sont disponibles
        for name in self.robot_names:
            if self.positions[name] is None:
                self.get_logger().info(f"Cannot plan yet, missing position for {name}")
                return False
        
        # Calculer les trajectoires directes (ligne droite de la position initiale à la cible)
        direct_paths = {}
        for name in self.robot_names:
            start = self.positions[name]
            end = self.targets[name]
            direct_paths[name] = (start, end)
            
            self.get_logger().info(f"Direct path for {name}: {start} -> {end}")
        
        # Vérifier les intersections des chemins directs
        intersections = self.find_intersections(direct_paths)
        
        if not intersections:
            self.get_logger().info("No path intersections detected, using direct paths")
            # Pas d'intersection, utiliser des lignes droites
            for name in self.robot_names:
                if self.use_bezier_curves:
                    self.trajectories[name] = self.generate_simple_bezier(direct_paths[name][0], direct_paths[name][1])
                else:
                    self.trajectories[name] = self.generate_waypoints(direct_paths[name][0], direct_paths[name][1])
        else:
            self.get_logger().info(f"Found {len(intersections)} potential collisions, planning alternative paths")
            # Planifier les détours pour éviter les collisions
            self.trajectories = self.create_detour_paths(direct_paths, intersections)
            
            # Vérifier que les nouvelles trajectoires ne se croisent pas
            self.verify_trajectories()
        
        # Initialiser les index des waypoints
        for name in self.robot_names:
            self.current_waypoint_index[name] = 0
            
        self.has_planned = True
        
        # Afficher les trajectoires planifiées pour le débogage
        for name, waypoints in self.trajectories.items():
            waypoints_str = ', '.join([f"({x:.2f}, {y:.2f})" for x, y in waypoints])
            self.get_logger().info(f"Trajectory for {name}: [{waypoints_str}]")
            
        # Publier les trajectoires planifiées
        self.publish_planned_trajectories()
            
        return True

    def constrain_to_arena(self, point):
        """Contraint un point à rester dans les limites de l'arène"""
        x = max(-2.0, min(2.0, point[0]))
        y = max(-1.0, min(0.8, point[1]))
        return (x, y)
    
    def create_detour_paths(self, direct_paths, intersections):
        """Crée des chemins avec des détours pour éviter les collisions selon la nouvelle logique"""
        # Dictionnaire pour stocker les points de détour pour chaque robot
        detour_points = {name: [] for name in self.robot_names}
        
        # Pour chaque intersection, calculer des points de détour
        for intersection in intersections:
            robot1, robot2 = intersection['robots']
            collision_point = intersection['point']
            
            # Points sur les trajectoires les plus proches de la collision
            point1 = intersection['point1']
            point2 = intersection['point2']
            
            # Déterminer les vecteurs de trajectoire pour chaque robot
            start1, end1 = direct_paths[robot1]
            start2, end2 = direct_paths[robot2]
            
            # Vecteurs directionnels des trajectoires
            dir1 = (end1[0] - start1[0], end1[1] - start1[1])
            dir2 = (end2[0] - start2[0], end2[1] - start2[1])
            
            # Normaliser les vecteurs directionnels
            len1 = math.hypot(dir1[0], dir1[1])
            len2 = math.hypot(dir2[0], dir2[1])
            
            if len1 > 0:
                dir1 = (dir1[0] / len1, dir1[1] / len1)
            if len2 > 0:
                dir2 = (dir2[0] / len2, dir2[1] / len2)
            
            # Vecteurs perpendiculaires aux trajectoires
            perp1 = (-dir1[1], dir1[0])
            perp2 = (-dir2[1], dir2[0])
            
            # Distance de sécurité pour le détour
            safe_distance = 2 * self.robot_radius + self.safety_margin
            
            # IMPORTANT: S'assurer que les vecteurs perpendiculaires pointent dans des directions opposées
            # Produit scalaire des vecteurs perpendiculaires
            dot_product = perp1[0] * perp2[0] + perp1[1] * perp2[1]
            
            # Si le produit scalaire est positif, les vecteurs pointent dans des directions similaires
            # Nous voulons qu'ils pointent dans des directions opposées
            if dot_product > 0:
                # Inverser l'un des vecteurs perpendiculaires
                perp2 = (-perp2[0], -perp2[1])
            
            # Calculer les points médians entre les points de départ et d'arrivée pour chaque robot
            mid_point1 = (
                (start1[0] + end1[0]) / 2,
                (start1[1] + end1[1]) / 2
            )
            
            mid_point2 = (
                (start2[0] + end2[0]) / 2,
                (start2[1] + end2[1]) / 2
            )
            
            # Calculer les vecteurs du point de collision vers les points médians
            vec_to_mid1 = (mid_point1[0] - collision_point[0], mid_point1[1] - collision_point[1])
            vec_to_mid2 = (mid_point2[0] - collision_point[0], mid_point2[1] - collision_point[1])
            
            # Calculer les points de détour équilibrés pour chaque robot
            # On combine le vecteur perpendiculaire (pour éviter la collision) 
            # avec un décalage vers le point médian (pour adoucir la courbe)
            detour1 = (
                point1[0] + safe_distance * perp1[0] + 0.3 * vec_to_mid1[0],
                point1[1] + safe_distance * perp1[1] + 0.3 * vec_to_mid1[1]
            )
            
            detour2 = (
                point2[0] + safe_distance * perp2[0] + 0.3 * vec_to_mid2[0],
                point2[1] + safe_distance * perp2[1] + 0.3 * vec_to_mid2[1]
            )
            
            # Contraindre les points de détour à rester dans les limites de l'arène
            detour1 = self.constrain_to_arena(detour1)
            detour2 = self.constrain_to_arena(detour2)
            
            # Calculer les paramètres t pour ordonner les détours le long des trajectoires
            t1 = self.calculate_path_parameter(start1, end1, point1)
            t2 = self.calculate_path_parameter(start2, end2, point2)
            
            # Ajouter les points de détour
            detour_points[robot1].append({
                'point': detour1,
                't': t1
            })
            
            detour_points[robot2].append({
                'point': detour2,
                't': t2
            })
            
            self.get_logger().info(f"Added detour for {robot1} at {detour1} (direction: {perp1}, midpoint influence: {vec_to_mid1})")
            self.get_logger().info(f"Added detour for {robot2} at {detour2} (direction: {perp2}, midpoint influence: {vec_to_mid2})")
        
        # Créer les trajectoires avec les détours
        trajectories = {}
        
        for name in self.robot_names:
            start = direct_paths[name][0]
            end = direct_paths[name][1]
            
            if not detour_points[name]:
                # Pas de détour pour ce robot, trajectoire directe
                if self.use_bezier_curves:
                    trajectories[name] = self.generate_simple_bezier(start, end)
                else:
                    trajectories[name] = self.generate_waypoints(start, end)
            else:
                # Trier les détours par ordre croissant de t
                detours = sorted(detour_points[name], key=lambda x: x['t'])
                
                # Créer une liste de points de contrôle pour la courbe de Bézier
                control_points = [start]
                
                for detour in detours:
                    detour_point = detour['point']
                    control_points.append(detour_point)
                
                control_points.append(end)
                
                # Générer la trajectoire lissée avec des courbes de Bézier
                if self.use_bezier_curves:
                    trajectories[name] = self.generate_enhanced_bezier(control_points)
                else:
                    # Si on n'utilise pas Bézier, utiliser des segments de ligne directe
                    waypoints = [start]
                    for detour in detours:
                        waypoints.append(detour['point'])
                    waypoints.append(end)
                    trajectories[name] = waypoints
        
        return trajectories
    
    def generate_enhanced_bezier(self, waypoints):
        """Génère une courbe de Bézier améliorée passant par les points fournis"""
        if len(waypoints) < 2:
            return waypoints
            
        if len(waypoints) == 2:
            # Ligne droite entre deux points
            return self.generate_simple_bezier(waypoints[0], waypoints[1])
        
        # Pour une courbe passant par plusieurs points, nous utilisons des courbes de Bézier par segments
        curve_points = []
        
        # Paramètres pour contrôler la forme de la courbe
        tension = 0.5  # Plus élevé = courbe plus tendue
        
        for i in range(len(waypoints) - 1):
            p0 = waypoints[i]
            p3 = waypoints[i+1]
            
            # Calculer les points de contrôle intermédiaires
            if i > 0:
                # Vector de p0 vers p3
                vec = (p3[0] - p0[0], p3[1] - p0[1])
                dist = math.hypot(vec[0], vec[1])
                
                # Point précédent pour calculer la tangente
                prev = waypoints[i-1]
                
                # Vecteur tangent (moyenne des directions)
                tan_x = tension * (p3[0] - prev[0])
                tan_y = tension * (p3[1] - prev[1])
                
                # Premier point de contrôle
                p1 = (p0[0] + tan_x / 3, p0[1] + tan_y / 3)
            else:
                # Premier segment, tangente parallèle au segment
                p1 = (p0[0] + (p3[0] - p0[0]) / 3, p0[1] + (p3[1] - p0[1]) / 3)
            
            if i < len(waypoints) - 2:
                # Vector de p0 vers p3
                vec = (p3[0] - p0[0], p3[1] - p0[1])
                dist = math.hypot(vec[0], vec[1])
                
                # Point suivant pour calculer la tangente
                next_p = waypoints[i+2]
                
                # Vecteur tangent (moyenne des directions)
                tan_x = tension * (next_p[0] - p0[0])
                tan_y = tension * (next_p[1] - p0[1])
                
                # Second point de contrôle
                p2 = (p3[0] - tan_x / 3, p3[1] - tan_y / 3)
            else:
                # Dernier segment, tangente parallèle au segment
                p2 = (p3[0] - (p3[0] - p0[0]) / 3, p3[1] - (p3[1] - p0[1]) / 3)
            
            # Générer les points de la courbe de Bézier cubique
            segment_points = []
            for t in np.linspace(0, 1, self.bezier_resolution if i == 0 else self.bezier_resolution // 2):
                # Formule de Bézier cubique
                x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
                y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
                segment_points.append((x, y))
            
            # Ajouter les points au résultat final (éviter le doublon avec le point de départ)
            if i == 0:
                curve_points.extend(segment_points)
            else:
                curve_points.extend(segment_points[1:])
        
        return curve_points

    def publish_planned_trajectories(self):
        """Publie les trajectoires planifiées pour visualisation"""
        if not self.has_planned:
            return
            
        # Préparer le message avec toutes les trajectoires
        msg = Float32MultiArray()
        
        # Format du message: [robot_count, r1_name_len, r1_name_chars..., r1_waypoint_count, r1_x1, r1_y1, r1_x2, r1_y2, ..., r2_name_len, ...]
        # Ça permet d'encoder toutes les trajectoires dans un seul tableau pour la transmission
        
        # Construire le tableau de données
        data = []
        data.append(float(len(self.robot_names)))  # Nombre de robots
        
        for name in self.robot_names:
            if name in self.trajectories and self.trajectories[name]:
                # Ajouter le nom du robot comme suite de codes ASCII
                name_chars = [float(ord(c)) for c in name]
                data.append(float(len(name_chars)))  # Longueur du nom
                data.extend(name_chars)
                
                # Ajouter les waypoints
                waypoints = self.trajectories[name]
                data.append(float(len(waypoints)))  # Nombre de waypoints
                for wp in waypoints:
                    data.append(float(wp[0]))  # x
                    data.append(float(wp[1]))  # y
            else:
                # Robot sans trajectoire
                name_chars = [float(ord(c)) for c in name]
                data.append(float(len(name_chars)))  # Longueur du nom
                data.extend(name_chars)
                data.append(0.0)  # 0 waypoints
                
        # Mettre à jour le message
        msg.data = data
        
        # Publier le message
        self.planned_trajectory_publisher.publish(msg)
        self.get_logger().info(f"Published planned trajectories for {len(self.robot_names)} robots")

    def find_intersections(self, paths):
        """Trouve les intersections entre les trajectoires des robots en tenant compte de leur taille"""
        intersections = []
        
        # Comparer chaque paire de chemins
        robot_names = list(paths.keys())
        for i in range(len(robot_names)):
            for j in range(i + 1, len(robot_names)):
                robot1 = robot_names[i]
                robot2 = robot_names[j]
                
                path1_start, path1_end = paths[robot1]
                path2_start, path2_end = paths[robot2]
                
                # Vérifier si les trajectoires passent trop près l'une de l'autre
                closest_point = self.find_closest_approach(
                    path1_start, path1_end,
                    path2_start, path2_end
                )
                
                if closest_point:
                    point1, point2, min_distance = closest_point
                    
                    # La distance minimale de sécurité est la somme des rayons des robots plus une marge
                    safe_distance = 2 * self.robot_radius + self.safety_margin
                    
                    if min_distance < safe_distance:
                        # Calculer le point médian de l'approche la plus proche
                        intersection_point = (
                            (point1[0] + point2[0]) / 2,
                            (point1[1] + point2[1]) / 2
                        )
                        
                        # Estimer le temps d'arrivée à ce point pour chaque robot
                        time1 = self.estimate_time_to_point(robot1, path1_start, point1)
                        time2 = self.estimate_time_to_point(robot2, path2_start, point2)
                        
                        # Si les robots arrivent à des moments proches (risque de collision)
                        if abs(time1 - time2) < 2.0:
                            intersections.append({
                                'point': intersection_point,
                                'point1': point1,
                                'point2': point2,
                                'robots': (robot1, robot2),
                                'times': (time1, time2),
                                'distance': min_distance
                            })
                            self.get_logger().info(
                                f"Potential collision detected between {robot1} and {robot2} "
                                f"at {intersection_point} with distance {min_distance:.2f}m "
                                f"(need {safe_distance:.2f}m)"
                            )
        
        return intersections

    def find_closest_approach(self, line1_start, line1_end, line2_start, line2_end):
        """
        Trouve les points d'approche les plus proches entre deux segments de ligne
        et retourne la distance minimale entre eux.
        """
        # Convertir en vecteurs de ligne
        u = (line1_end[0] - line1_start[0], line1_end[1] - line1_start[1])
        v = (line2_end[0] - line2_start[0], line2_end[1] - line2_start[1])
        
        # Vecteur entre les points de départ
        w = (line1_start[0] - line2_start[0], line1_start[1] - line2_start[1])
        
        # Coefficients pour le calcul
        a = u[0]*u[0] + u[1]*u[1]  # |u|^2
        b = u[0]*v[0] + u[1]*v[1]  # u·v
        c = v[0]*v[0] + v[1]*v[1]  # |v|^2
        d = u[0]*w[0] + u[1]*w[1]  # u·w
        e = v[0]*w[0] + v[1]*w[1]  # v·w
        
        # Dénominateur
        denominator = a*c - b*b
        
        # Si les lignes sont presque parallèles, vérifier les extrémités
        if abs(denominator) < 1e-6:
            # Vérifier les 4 distances extremité-à-extremité
            distances = [
                (self.distance(line1_start, line2_start), line1_start, line2_start),
                (self.distance(line1_start, line2_end), line1_start, line2_end),
                (self.distance(line1_end, line2_start), line1_end, line2_start),
                (self.distance(line1_end, line2_end), line1_end, line2_end)
            ]
            min_dist, p1, p2 = min(distances, key=lambda x: x[0])
            return (p1, p2, min_dist)
        
        # Paramètres pour les points les plus proches
        s = (b*e - c*d) / denominator
        t = (a*e - b*d) / denominator
        
        # Limiter s et t entre 0 et 1 (pour rester sur les segments)
        s = max(0, min(1, s))
        t = max(0, min(1, t))
        
        # Calculer les points d'approche les plus proches
        point1 = (
            line1_start[0] + s * u[0],
            line1_start[1] + s * u[1]
        )
        
        point2 = (
            line2_start[0] + t * v[0],
            line2_start[1] + t * v[1]
        )
        
        # Calculer la distance minimale
        min_distance = self.distance(point1, point2)
        
        return (point1, point2, min_distance)
    
    def distance(self, point1, point2):
        """Calcule la distance euclidienne entre deux points"""
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

    def line_intersection(self, line1_start, line1_end, line2_start, line2_end):
        """
        Cette méthode est maintenue pour compatibilité, mais nous utilisons maintenant
        find_closest_approach qui est plus robuste pour détecter les risques de collision.
        """
        x1, y1 = line1_start
        x2, y2 = line1_end
        x3, y3 = line2_start
        x4, y4 = line2_end
        
        # Calculer les coefficients de l'équation de la ligne
        A1 = y2 - y1
        B1 = x1 - x2
        C1 = A1 * x1 + B1 * y1
        
        A2 = y4 - y3
        B2 = x3 - x4
        C2 = A2 * x3 + B2 * y3
        
        # Calculer le déterminant
        det = A1 * B2 - A2 * B1
        
        if abs(det) < 1e-6:  # Les lignes sont parallèles
            return None
        
        # Calculer le point d'intersection
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        
        # Vérifier si l'intersection est sur les segments
        if (min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= max(y1, y2) and
            min(x3, x4) <= x <= max(x3, x4) and min(y3, y4) <= max(y3, y4)):
            return (x, y)
        
        return None

    def estimate_time_to_point(self, robot_name, start, point):
        """Estime le temps nécessaire pour qu'un robot atteigne un point"""
        distance = math.hypot(point[0] - start[0], point[1] - start[1])
        # Estimation approximative basée sur la vitesse normale
        return distance / self.normal_speed

    def calculate_path_parameter(self, start, end, point):
        """Calcule le paramètre t (0-1) pour un point par rapport à une ligne segment"""
        # Vecteur de la ligne
        line_x = end[0] - start[0]
        line_y = end[1] - start[1]
        
        # Vecteur du point au début de la ligne
        point_x = point[0] - start[0]
        point_y = point[1] - start[1]
        
        # Projection du point sur la ligne
        line_length_squared = line_x * line_x + line_y * line_y
        
        if line_length_squared < 1e-6:  # Ligne trop courte
            return 0
            
        t = (point_x * line_x + point_y * line_y) / line_length_squared
        
        # Limiter t entre 0 et 1
        t = max(0, min(1, t))
        return t

    def generate_waypoints(self, start, end):
        """Génère une séquence de waypoints entre deux points"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        
        # Si la distance est très faible, retourner simplement les deux points
        if distance < self.waypoint_spacing:
            return [start, end]
        
        # Nombre de waypoints intermédiaires
        num_waypoints = int(distance / self.waypoint_spacing) + 1
        
        waypoints = [start]
        for i in range(1, num_waypoints):
            t = i / num_waypoints
            x = start[0] + t * dx
            y = start[1] + t * dy
            waypoints.append((x, y))
        
        # Ajouter le point final (même si on dépasse un peu num_waypoints)
        if waypoints[-1] != end:
            waypoints.append(end)
            
        return waypoints

    def plan_alternative_paths(self, direct_paths, intersections):
        """Planifie des chemins alternatifs pour éviter les collisions avec des courbes douces"""
        # Dictionnaire qui stockera les trajectoires finales
        trajectories = {}
        
        # Dictionnaire pour stocker les points de détour pour chaque robot
        detour_points = {name: [] for name in self.robot_names}
        
        # Pour chaque intersection, déterminer le robot qui doit éviter l'autre
        for intersection in intersections:
            point = intersection['point']
            robot1, robot2 = intersection['robots']
            time1, time2 = intersection['times']
            
            # Déterminer quel robot doit faire un détour pour éviter l'autre
            if abs(time1 - time2) > 0.5:  # Différence de temps significative
                avoiding_robot = robot1 if time1 > time2 else robot2
                direct_robot = robot2 if time1 > time2 else robot1
            else:  # Temps similaire, priorité alphabétique
                avoiding_robot = robot2 if robot1 < robot2 else robot1
                direct_robot = robot1 if robot1 < robot2 else robot2
                
            self.get_logger().info(f"At intersection {point}, {avoiding_robot} will avoid {direct_robot}")
            
            # Calculer le point de détour pour le robot qui évite
            start = direct_paths[avoiding_robot][0]
            end = direct_paths[avoiding_robot][1]
            
            # Point d'intersection projeté sur la trajectoire du robot qui évite
            if avoiding_robot == robot1:
                intersection_projected = intersection['point1']
            else:
                intersection_projected = intersection['point2']
            
            # Déterminer la direction perpendiculaire à la trajectoire pour le détour
            # Vecteur de la trajectoire
            path_vector = (end[0] - start[0], end[1] - start[1])
            path_length = math.hypot(path_vector[0], path_vector[1])
            
            if path_length > 0:
                # Normaliser
                path_dir = (path_vector[0] / path_length, path_vector[1] / path_length)
                
                # Vecteur perpendiculaire (rotation de 90°)
                perp_dir = (-path_dir[1], path_dir[0])
                
                # Distance de sécurité pour le détour
                detour_distance = 2 * self.robot_radius + self.safety_margin * 2
                
                # Calculer le point de détour
                detour_point = (
                    intersection_projected[0] + detour_distance * perp_dir[0],
                    intersection_projected[1] + detour_distance * perp_dir[1]
                )
                
                # Contraindre le point de détour aux limites de l'arène
                detour_point = self.constrain_to_arena(detour_point)
                
                # Calculer le paramètre t de l'intersection le long de la trajectoire
                t = self.calculate_path_parameter(start, end, intersection_projected)
                
                # Ajouter le point de détour
                detour_points[avoiding_robot].append({
                    'intersection': intersection_projected,
                    'detour_point': detour_point,
                    't': t  # Paramètre pour trier les détours dans l'ordre
                })
                
                self.get_logger().info(f"Added detour for {avoiding_robot}: {detour_point}")
        
        # Pour chaque robot, générer sa trajectoire
        for name in self.robot_names:
            start = direct_paths[name][0]
            end = direct_paths[name][1]
            
            if not detour_points[name]:  # Pas de détour pour ce robot
                # Utiliser un chemin direct
                if self.use_bezier_curves:
                    trajectories[name] = self.generate_simple_bezier(start, end)
                else:
                    trajectories[name] = self.generate_waypoints(start, end)
            else:
                # Trier les points de détour par t croissant
                sorted_detours = sorted(detour_points[name], key=lambda x: x['t'])
                
                if self.use_bezier_curves:
                    # Générer une courbe de Bézier avec les points de détour
                    control_points = [start]
                    
                    for detour in sorted_detours:
                        # Ajouter des points de contrôle pour une courbe lisse
                        intersection = detour['intersection']
                        detour_point = detour['detour_point']
                        
                        # Vecteur de l'intersection au détour
                        dx = detour_point[0] - intersection[0]
                        dy = detour_point[1] - intersection[1]
                        
                        # Points de contrôle avant et après pour donner de la courbure
                        control1 = (
                            intersection[0] - dx * self.bezier_control_scale,
                            intersection[1] - dy * self.bezier_control_scale
                        )
                        control2 = (
                            detour_point[0],
                            detour_point[1]
                        )
                        control3 = (
                            detour_point[0] + dx * self.bezier_control_scale,
                            detour_point[1] + dy * self.bezier_control_scale
                        )
                        
                        control_points.extend([control1, control2, control3])
                    
                    # Ajouter le point final
                    control_points.append(end)
                    
                    # Générer la courbe
                    trajectories[name] = self.generate_bezier_curve(control_points)
                else:
                    # Générer des waypoints segmentés
                    waypoints = [start]
                    current = start
                    
                    for detour in sorted_detours:
                        # Ajouter des waypoints vers le point de détour
                        detour_waypoints = self.generate_waypoints(current, detour['detour_point'])
                        waypoints.extend(detour_waypoints[1:])  # Éviter de dupliquer le point de départ
                        current = detour['detour_point']
                    
                    # Ajouter des waypoints vers la destination finale
                    final_waypoints = self.generate_waypoints(current, end)
                    waypoints.extend(final_waypoints[1:])  # Éviter de dupliquer le dernier point
                    
                    trajectories[name] = waypoints
        
        return trajectories

    def generate_bezier_curve(self, control_points):
        """Génère une courbe de Bézier avec plusieurs points de contrôle (approximation)"""
        if len(control_points) < 2:
            return control_points
            
        if len(control_points) == 2:
            # Simple ligne droite
            return self.generate_waypoints(control_points[0], control_points[1])
        
        # Pour les courbes de Bézier complexes, nous utilisons une approximation
        # en la décomposant en plusieurs courbes de Bézier cubiques
        
        # Diviser les points de contrôle en segments de courbes cubiques
        segments = []
        for i in range(0, len(control_points) - 1, 3):
            if i + 3 < len(control_points):
                segments.append([control_points[i], control_points[i+1], control_points[i+2], control_points[i+3]])
            elif i + 2 < len(control_points):
                # Dernier segment incomplet - utiliser une courbe quadratique
                segments.append([control_points[i], control_points[i+1], control_points[i+2]])
            else:
                # Segment final - ligne droite
                segments.append([control_points[i], control_points[i+1]])
        
        # Générer points pour chaque segment
        curve_points = []
        total_segments = len(segments)
        
        for idx, segment in enumerate(segments):
            # Distribution des points pour que les segments plus longs aient plus de points
            points_per_segment = max(5, int(self.bezier_resolution / total_segments))
            
            if len(segment) == 4:  # Courbe cubique
                for t in np.linspace(0, 1, points_per_segment):
                    p0, p1, p2, p3 = segment
                    # Formule de Bézier cubique
                    x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
                    y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
                    # N'ajouter le point que s'il n'est pas déjà présent (éviter les doublons)
                    if not curve_points or (x, y) != curve_points[-1]:
                        curve_points.append((x, y))
                        
            elif len(segment) == 3:  # Courbe quadratique
                for t in np.linspace(0, 1, points_per_segment):
                    p0, p1, p2 = segment
                    # Formule de Bézier quadratique
                    x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
                    y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
                    if not curve_points or (x, y) != curve_points[-1]:
                        curve_points.append((x, y))
                        
            else:  # Ligne droite
                p0, p1 = segment
                for t in np.linspace(0, 1, points_per_segment):
                    x = (1-t) * p0[0] + t * p1[0]
                    y = (1-t) * p0[1] + t * p1[1]
                    if not curve_points or (x, y) != curve_points[-1]:
                        curve_points.append((x, y))
        
        return curve_points

    def verify_trajectories(self):
        """Vérifie que les trajectoires recalculées ne se croisent pas"""
        # Créer des copies temporaires des trajectoires pour les tests
        collision_detected = False
        
        # Vérifier les collisions potentielles entre les trajectoires
        for i in range(len(self.robot_names)):
            for j in range(i + 1, len(self.robot_names)):
                robot1 = self.robot_names[i]
                robot2 = self.robot_names[j]
                
                trajectory1 = self.trajectories[robot1]
                trajectory2 = self.trajectories[robot2]
                
                # Convertir en segments pour la vérification
                segments1 = [(trajectory1[k], trajectory1[k+1]) for k in range(len(trajectory1)-1)]
                segments2 = [(trajectory2[k], trajectory2[k+1]) for k in range(len(trajectory2)-1)]
                
                for seg1 in segments1:
                    for seg2 in segments2:
                        closest = self.find_closest_approach(seg1[0], seg1[1], seg2[0], seg2[1])
                        
                        if closest:
                            _, _, distance = closest
                            safe_distance = 2 * self.robot_radius + self.safety_margin
                            
                            if distance < safe_distance:
                                collision_detected = True
                                self.get_logger().warning(
                                    f"Warning: Trajectories for {robot1} and {robot2} still have "
                                    f"a close approach with distance {distance:.2f}m < {safe_distance:.2f}m"
                                )
        
        if collision_detected:
            self.get_logger().warning("Replanning failed to eliminate all collisions. Adding more clearance.")
            # Augmenter la marge de sécurité et réessayer
            self.safety_margin += 0.1
            # Note: Dans une implémentation plus complète, on pourrait réitérer la planification ici
        else:
            self.get_logger().info("All recalculated trajectories are collision-free.")

    def generate_simple_bezier(self, start, end):
        """Génère une courbe de Bézier simple (quadratique) entre deux points"""
        # Calculer le point de contrôle au milieu
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2
        
        # Générer les points de la courbe
        points = []
        for t in np.linspace(0, 1, self.bezier_resolution):
            # Formule de la courbe de Bézier quadratique
            x = (1-t)**2 * start[0] + 2*(1-t)*t * mid_x + t**2 * end[0]
            y = (1-t)**2 * start[1] + 2*(1-t)*t * mid_y + t**2 * end[1]
            points.append((x, y))
        
        return points

    def follow_trajectory(self, name, position):
        """Suit la trajectoire avec anticipation pour plus de fluidité"""
        if not self.has_planned and name not in self.trajectories:
            return None, None, 0.0, False
        
        trajectory = self.trajectories[name]
        index = self.current_waypoint_index[name]
        
        if index >= len(trajectory):
            return self.targets[name], (0, 0), 0.0, True
        
        # Calculer la distance au waypoint actuel
        target = trajectory[index]
        distance = math.hypot(target[0] - position[0], target[1] - position[1])
        is_final_target = (index == len(trajectory) - 1)
        
        # Si on est assez proche du waypoint, passer au suivant
        if distance < self.waypoint_tolerance and index < len(trajectory) - 1:
            self.current_waypoint_index[name] += 1
            index = self.current_waypoint_index[name]
            target = trajectory[index]
            distance = math.hypot(target[0] - position[0], target[1] - position[1])
            is_final_target = (index == len(trajectory) - 1)
            
        # Technique de regard en avant (lookahead) pour un suivi de trajectoire plus fluide
        # Chercher un point plus loin sur la trajectoire pour anticiper les virages
        if not is_final_target and index + 1 < len(trajectory):
            # Calculer le point de la trajectoire qui est à lookahead_distance du robot
            cumulative_distance = distance
            lookahead_index = index
            
            while cumulative_distance < self.lookahead_distance and lookahead_index + 1 < len(trajectory):
                lookahead_index += 1
                next_point = trajectory[lookahead_index]
                segment_distance = math.hypot(
                    next_point[0] - trajectory[lookahead_index-1][0], 
                    next_point[1] - trajectory[lookahead_index-1][1]
                )
                cumulative_distance += segment_distance
            
            if lookahead_index > index:
                # Utiliser un point interpolé entre les waypoints pour un suivi plus doux
                target = trajectory[lookahead_index]
        
        # Calculer l'erreur de position
        error_x = target[0] - position[0]
        error_y = target[1] - position[1]
        
        return target, (error_x, error_y), distance, is_final_target

    def adjust_speed_for_waypoint(self, target, position, is_final_target):
        """Ajuste la vitesse de manière plus progressive"""
        distance = math.hypot(target[0] - position[0], target[1] - position[1])
        speed_factor = 1.0
        
        # Pour le waypoint final, ralentir progressivement avec une courbe plus douce
        if is_final_target:
            if distance < self.final_approach_distance:
                # Utiliser une fonction quadratique pour une décélération plus douce
                ratio = distance / self.final_approach_distance
                speed_factor = 0.3 + 0.7 * (ratio ** 1.5)  # Exposant > 1 pour courbe plus douce
        else:
            # Pour les waypoints intermédiaires, utiliser une transition plus progressive
            if distance < self.waypoint_tolerance * 4:
                ratio = distance / (self.waypoint_tolerance * 4)
                speed_factor = max(self.waypoint_slowdown, ratio ** 0.8)  # Exposant < 1 pour transition douce
        
        return speed_factor

    def smooth_command(self, name, cmd_x, cmd_y):
        """Applique un filtre de lissage aux commandes pour éviter les changements brusques"""
        prev_x, prev_y = self.previous_commands[name]
        
        # Mixage de la commande actuelle avec la précédente pour un mouvement plus fluide
        smooth_x = self.smoothing_factor * prev_x + (1 - self.smoothing_factor) * cmd_x
        smooth_y = self.smoothing_factor * prev_y + (1 - self.smoothing_factor) * cmd_y
        
        # Limiter l'accélération
        dx = smooth_x - prev_x
        dy = smooth_y - prev_y
        
        # Si l'accélération est trop importante, la limiter
        acceleration = math.hypot(dx, dy)
        if (acceleration > self.max_acceleration):
            # Normaliser et mettre à l'échelle
            factor = self.max_acceleration / acceleration
            dx *= factor
            dy *= factor
            smooth_x = prev_x + dx
            smooth_y = prev_y + dy
        
        # Enregistrer pour la prochaine itération
        self.previous_commands[name] = (smooth_x, smooth_y)
        
        return smooth_x, smooth_y

    def control_loop(self):
        """Boucle principale de contrôle améliorée pour plus de fluidité"""
        all_robots_on_target = True
        
        # Si les trajectoires ne sont pas encore planifiées, essayer de les planifier
        if not self.has_planned:
            # D'abord, vérifier si tous les robots ont l'orientation initiale correcte
            if self.align_all_robots_initially():
                # Une fois que tous les robots sont alignés, planifier les trajectoires
                if not self.plan_trajectories():
                    return  # Attendre d'avoir toutes les positions
            else:
                # Si les robots ne sont pas encore alignés, continuer le processus d'alignement
                return
                
        current_time = self.get_clock().now()

        for i, name in enumerate(self.robot_names):
            position = self.positions[name]
            if position is None:
                all_robots_on_target = False
                continue  # On n'a pas encore la pose de ce robot
            
            # Vérifier si le robot est en phase d'orientation finale
            if self.orientation_phase[name] == "final":
                # Dans ce cas, on oriente le robot sans le déplacer
                if self.align_robot_orientation(name, i):
                    # Robot aligné correctement à la position finale
                    continue
                else:
                    all_robots_on_target = False
                    continue
            
            # Vérifier si le robot est bloqué
            if self.check_if_robot_stuck(name, position, current_time):
                # Robot bloqué, recalculer sa trajectoire
                self.recalculate_robot_trajectory(name)
            
            # Suivre la trajectoire planifiée
            target, errors, distance, is_final_target = self.follow_trajectory(name, position)
            
            # Si on n'a pas de cible valide, utiliser directement la cible finale
            if target is None:
                target = self.targets[name]
                errors = (target[0] - position[0], target[1] - position[1])
                distance = math.hypot(errors[0], errors[1])
                is_final_target = True
            
            cmd = Twist()
            
            # Si on est suffisamment proche de la cible finale
            if distance <= 0.05 and is_final_target:
                # Passer à la phase d'orientation finale
                if self.orientation_phase[name] != "final":
                    self.orientation_phase[name] = "final"
                    self.get_logger().info(f"Robot {name} has reached target position, aligning orientation")
                
                # Le contrôle de l'orientation se fait maintenant dans align_robot_orientation
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                # Réinitialiser la commande précédente pour éviter les sursauts lors du redémarrage
                self.previous_commands[name] = (0.0, 0.0)
                
                all_robots_on_target = False  # On considère que le robot n'est pas sur cible tant qu'il n'est pas bien orienté
            else:
                all_robots_on_target = False
                self.orientation_phase[name] = "moving"  # Robot en mouvement
                
                # Ajuster la vitesse en fonction de la distance au waypoint
                speed_factor = self.adjust_speed_for_waypoint(target, position, is_final_target)
                
                # Contrôleur PD simple
                cmd_x = self.k_p * errors[0] - self.k_d * self.velocities[name][0]
                cmd_y = self.k_p * errors[1] - self.k_d * self.velocities[name][1]
                
                # Appliquer le facteur de vitesse
                cmd_x *= speed_factor
                cmd_y *= speed_factor
                
                # Appliquer le lissage pour des transitions fluides
                cmd_x, cmd_y = self.smooth_command(name, cmd_x, cmd_y)
                
                cmd.linear.x = cmd_x
                cmd.linear.y = cmd_y
                
                # Limiter la vitesse maximale
                speed = math.hypot(cmd.linear.x, cmd.linear.y)
                if speed > self.normal_speed:
                    cmd.linear.x = (cmd.linear.x / speed) * self.normal_speed
                    cmd.linear.y = (cmd.linear.y / speed) * self.normal_speed
                
                # Si le robot est en mouvement, pas de contrôle d'orientation
                cmd.angular.z = 0.0
            
            # Publier la commande
            self.cmd_publishers[i].publish(cmd)
        
        # Vérifier si tous les robots ont atteint leurs cibles et sont correctement orientés
        if all_robots_on_target:
            # Vérifier si tous les robots sont correctement orientés
            all_robots_oriented = True
            for name in self.robot_names:
                if abs(self.normalize_angle(self.orientations[name] - self.target_orientation)) > self.orientation_tolerance:
                    all_robots_oriented = False
                    break
                    
            if all_robots_oriented:
                self.get_logger().info("Formation complete with correct orientation! Program is stopped")
                self.formation_complete = True
                rclpy.shutdown()

    def align_all_robots_initially(self):
        """Aligne tous les robots à l'orientation cible avant de commencer à se déplacer"""
        all_aligned = True
        
        for i, name in enumerate(self.robot_names):
            # Vérifier si l'orientation est disponible
            if self.orientations[name] is None:
                all_aligned = False
                continue
                
            if self.orientation_phase[name] != "initial":
                continue  # Ce robot a déjà terminé sa phase d'alignement initial
                
            # Vérifier si l'orientation est proche de la cible
            angle_diff = self.normalize_angle(self.orientations[name] - self.target_orientation)
            
            if abs(angle_diff) <= self.orientation_tolerance:
                # Robot aligné, marquer comme prêt
                self.orientation_phase[name] = "moving"
                self.get_logger().info(f"Robot {name} initial orientation aligned")
                continue
                
            # Robot non aligné, appliquer une commande de rotation
            all_aligned = False
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            
            # Calculer la vitesse angulaire proportionnelle à l'erreur d'angle
            k_angle = 0.5  # Gain pour le contrôle angulaire
            cmd.angular.z = -k_angle * angle_diff  # Négatif car sens horaire est négatif
            
            # Limiter la vitesse angulaire
            max_angular_vel = 0.5  # rad/s
            cmd.angular.z = max(-max_angular_vel, min(max_angular_vel, cmd.angular.z))
            
            # Publier la commande
            self.cmd_publishers[i].publish(cmd)
            
        return all_aligned

    def align_robot_orientation(self, name, index):
        """Aligne un robot à l'orientation cible une fois qu'il a atteint sa position finale"""
        # Vérifier si l'orientation est disponible
        if self.orientations[name] is None:
            return False
            
        # Vérifier si l'orientation est proche de la cible
        angle_diff = self.normalize_angle(self.orientations[name] - self.target_orientation)
        
        if abs(angle_diff) <= self.orientation_tolerance:
            # Robot aligné
            return True
            
        # Robot non aligné, appliquer une commande de rotation
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        
        # Calculer la vitesse angulaire proportionnelle à l'erreur d'angle
        k_angle = 0.5  # Gain pour le contrôle angulaire
        cmd.angular.z = -k_angle * angle_diff  # Négatif car sens horaire est négatif
        
        # Limiter la vitesse angulaire et la rendre plus douce près de la cible
        max_angular_vel = 0.5  # rad/s
        if abs(angle_diff) < 0.3:  # Proche de la cible
            cmd.angular.z *= 0.7  # Réduire la vitesse pour un arrêt plus doux
            
        cmd.angular.z = max(-max_angular_vel, min(max_angular_vel, cmd.angular.z))
        
        # Publier la commande
        self.cmd_publishers[index].publish(cmd)
        
        return False

    def normalize_angle(self, angle):
        """Normalise un angle entre -pi et pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def check_if_robot_stuck(self, name, current_position, current_time):
        """Vérifie si un robot est bloqué (ne progresse pas vers son waypoint)"""
        # Ignorer si nous sommes en période de refroidissement après une recalculation
        if self.trajectory_recalculation_cooldown[name] > 0:
            self.trajectory_recalculation_cooldown[name] -= 1
            return False
        
        # Vérifier si le robot est déjà proche de sa position cible finale
        target_position = self.targets[name]
        distance_to_target = math.hypot(
            target_position[0] - current_position[0],
            target_position[1] - current_position[1]
        )
        
        # Si le robot est déjà très proche de sa cible finale, ne pas le considérer comme bloqué
        if distance_to_target <= 0.05:
            return False
            
        # Initialiser les positions de référence si nécessaire
        if self.last_progress_positions[name] is None:
            self.last_progress_positions[name] = current_position
            self.last_progress_time[name] = current_time
            return False
            
        # Calculer le temps écoulé depuis la dernière vérification
        time_diff = (current_time - self.last_progress_time[name]).nanoseconds / 1e9
        
        # Vérifier seulement après un intervalle de temps suffisant
        if time_diff > self.stuck_detection_time:
            # Calculer la distance parcourue depuis la dernière vérification
            distance_moved = math.hypot(
                current_position[0] - self.last_progress_positions[name][0],
                current_position[1] - self.last_progress_positions[name][1]
            )
            
            # Mettre à jour la position et le temps de référence
            self.last_progress_positions[name] = current_position
            self.last_progress_time[name] = current_time
            
            # Si le robot ne s'est pas assez déplacé, le considérer comme bloqué
            if distance_moved < self.stuck_distance_threshold:
                self.robot_stuck_count[name] += 1
                self.get_logger().warning(f"Robot {name} seems stuck (attempt {self.robot_stuck_count[name]})")
                return self.robot_stuck_count[name] <= self.max_recalculations
            else:
                # Réinitialiser le compteur de blocage si le robot bouge normalement
                self.robot_stuck_count[name] = 0
                
        return False

    def recalculate_robot_trajectory(self, name):
        """Recalcule la trajectoire pour un robot bloqué"""
        self.get_logger().info(f"Recalculating trajectory for {name}")
        
        current_position = self.positions[name]
        target_position = self.targets[name]
        
        if current_position is None:
            return False
            
        # Augmenter la marge de sécurité pour éviter de reproduire le même blocage
        temp_safety_margin = self.safety_margin
        self.safety_margin += 0.05 * self.robot_stuck_count[name]
        
        # Récupérer l'index du waypoint actuel
        current_index = self.current_waypoint_index[name]
        
        # Si nous sommes déjà proche de la destination finale, simplifier la trajectoire
        distance_to_target = math.hypot(
            target_position[0] - current_position[0],
            target_position[1] - current_position[1]
        )
        
        if distance_to_target < 0.5:  # Si nous sommes assez proches de la cible
            # Générer une trajectoire directe avec une légère courbe
            midpoint_x = (current_position[0] + target_position[0]) / 2
            midpoint_y = (current_position[1] + target_position[1]) / 2
            
            # Ajouter un petit décalage aléatoire pour éviter de refaire la même trajectoire
            offset_x = 0.1 * (2 * np.random.random() - 1)
            offset_y = 0.1 * (2 * np.random.random() - 1)
            
            midpoint = (midpoint_x + offset_x, midpoint_y + offset_y)
            midpoint = self.constrain_to_arena(midpoint)
            
            if self.use_bezier_curves:
                # Générer une courbe de Bézier qui passe par le milieu modifié
                control_points = [current_position, midpoint, target_position]
                self.trajectories[name] = self.generate_enhanced_bezier(control_points)
            else:
                # Générer des waypoints directs
                self.trajectories[name] = [current_position, midpoint, target_position]
        else:
            # Pour les distances plus grandes, essayer de contourner l'obstacle
            
            # Déterminer un angle de déviation basé sur le nombre de tentatives
            deviation_angle = np.pi/4 * (1 + 0.5 * (self.robot_stuck_count[name] % 4))
            if self.robot_stuck_count[name] % 2 == 0:
                deviation_angle = -deviation_angle
                
            # Calculer le vecteur direction vers la cible
            direction = (
                target_position[0] - current_position[0],
                target_position[1] - current_position[1]
            )
            dir_length = math.hypot(direction[0], direction[1])
            
            if dir_length > 0:
                # Normaliser
                direction = (direction[0] / dir_length, direction[1] / dir_length)
                
                # Calculer le vecteur dévié
                cos_a = math.cos(deviation_angle)
                sin_a = math.sin(deviation_angle)
                deviated_dir = (
                    direction[0] * cos_a - direction[1] * sin_a,
                    direction[0] * sin_a + direction[1] * cos_a
                )
                
                # Créer un point intermédiaire à distance moyenne dans la direction déviée
                detour_distance = min(0.6, distance_to_target * 0.6)
                detour_point = (
                    current_position[0] + detour_distance * deviated_dir[0],
                    current_position[1] + detour_distance * deviated_dir[1]
                )
                
                # S'assurer que le point est dans les limites de l'arène
                detour_point = self.constrain_to_arena(detour_point)
                
                # Générer une nouvelle trajectoire
                if self.use_bezier_curves:
                    control_points = [current_position, detour_point, target_position]
                    self.trajectories[name] = self.generate_enhanced_bezier(control_points)
                else:
                    self.trajectories[name] = [current_position, detour_point, target_position]
        
        # Réinitialiser l'index de waypoint
        self.current_waypoint_index[name] = 0
        
        # Appliquer un temps de refroidissement pour éviter des recalculations trop fréquentes
        self.trajectory_recalculation_cooldown[name] = 10  # Attendre 10 cycles avant de vérifier à nouveau
        
        # Restaurer la valeur originale de la marge de sécurité
        self.safety_margin = temp_safety_margin
        
        # Publier les trajectoires mises à jour
        self.publish_planned_trajectories()
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = FormationInitializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
