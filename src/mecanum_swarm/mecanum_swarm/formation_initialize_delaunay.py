import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import numpy as np
from typing import Dict, List, Tuple
from mecanum_swarm.config import ALL_ROBOT_NAMES
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class FormationGraph:
    """Représente la formation comme un graphe avec les relations de voisinage"""
    
    def __init__(self):
        self.nodes = {}  # nom_robot -> position
        self.neighbors = {}  # nom_robot -> liste des noms des voisins
        self.edges = []  # liste des tuples (robot1, robot2)
    
    def add_robot(self, robot_name: str, position: Tuple[float, float]):
        """Ajouter un robot au graphe"""
        self.nodes[robot_name] = position
        if robot_name not in self.neighbors:
            self.neighbors[robot_name] = []
    
    def add_neighbor_relationship(self, robot1: str, robot2: str):
        """Ajouter une relation de voisinage bidirectionnelle"""
        if robot2 not in self.neighbors[robot1]:
            self.neighbors[robot1].append(robot2)
        if robot1 not in self.neighbors[robot2]:
            self.neighbors[robot2].append(robot1)
        
        edge = tuple(sorted([robot1, robot2]))
        if edge not in self.edges:
            self.edges.append(edge)
    
    def get_neighbors(self, robot_name: str) -> List[str]:
        """Obtenir la liste des voisins d'un robot"""
        return self.neighbors.get(robot_name, [])
    
    def print_formation(self):
        """Afficher la structure de la formation"""
        print("\n=== Graphe de Formation (Delaunay) ===")
        for robot, neighbors in self.neighbors.items():
            pos = self.nodes[robot]
            print(f"{robot} à ({pos[0]:.2f}, {pos[1]:.2f}) -> voisins: {neighbors}")
        print(f"Total d'arêtes: {len(self.edges)}")

class FormationInitializer(Node):
    def __init__(self):
        super().__init__('formation_initializer')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.formation_graph = FormationGraph()
        self.neighbor_distance_threshold = self.declare_parameter('neighbor_distance_threshold', 0.8).value  # mètres
        
        # Paramètres simplifiés
        self.allow_diagonal_neighbors = self.declare_parameter('allow_diagonal_neighbors', False).value
        self.enable_plotting = self.declare_parameter('enable_plotting', False).value
        
        # Seuil d'alignement
        self.alignment_tolerance = self.declare_parameter('alignment_tolerance', 0.1).value  # mètres
        
        # Paramètre pour activer l'écriture YAML
        self.write_to_yaml = self.declare_parameter('write_to_yaml', True).value
        
        # Chemin par défaut basé sur le package ROS2
        try:
            package_share_dir = get_package_share_directory('mecanum_swarm')
            default_yaml_path = os.path.join(package_share_dir, 'config', 'robots.yaml')
        except:
            # Fallback vers un chemin relatif si le package n'est pas trouvé
            default_yaml_path = os.path.join(os.path.expanduser('~'), 'mecanum', 'src', 'mecanum_swarm', 'config', 'robots.yaml')
        
        self.yaml_file_path = self.declare_parameter('yaml_file_path', default_yaml_path).value
        
        # Seuil maximal pour le rayon du cercle circonscrit (en mètres)
        self.max_circumcircle_radius = self.declare_parameter('max_circumcircle_radius', 0.8).value
        
        # Variables pour le traçage
        self.figure = None
        
        # Timer pour mettre à jour la formation
        self.create_timer(1.0, self.update_formation)  # 1 Hz
        
        self.get_logger().info("Initialisateur de Formation démarré - Méthode: Triangulation de Delaunay")
        self.get_logger().info(f"Seuil de distance: {self.neighbor_distance_threshold}m")
        self.get_logger().info(f"Tolérance d'alignement: {self.alignment_tolerance}m")
        self.get_logger().info(f"Voisins diagonaux autorisés: {self.allow_diagonal_neighbors}")
        
        if self.enable_plotting:
            self.get_logger().info("Visualisation graphique activée")
            plt.ion()
            plt.show()
    
    def get_robot_positions(self) -> Dict[str, Tuple[float, float]]:
        """Obtenir les positions actuelles de tous les robots depuis TF"""
        positions = {}
        
        for robot_name in ALL_ROBOT_NAMES:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'mocap',
                    f'{robot_name}/base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                positions[robot_name] = (x, y)
                
            except Exception as e:
                self.get_logger().debug(f"Impossible d'obtenir la transformation pour {robot_name}: {e}")
                continue
        
        return positions
    
    def calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculer la distance euclidienne entre deux positions"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def is_aligned(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> bool:
        """Vérifier si deux robots sont alignés horizontalement ou verticalement"""
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        
        is_horizontally_aligned = dy < self.alignment_tolerance
        is_vertically_aligned = dx < self.alignment_tolerance
        
        aligned = is_horizontally_aligned or is_vertically_aligned
        
        if aligned:
            alignment_type = "horizontal" if is_horizontally_aligned else "vertical"
            self.get_logger().debug(f"Robots alignés {alignment_type}: dx={dx:.3f}, dy={dy:.3f}")
        
        return aligned
    
    def find_delaunay_neighbors(self, positions: Dict[str, Tuple[float, float]]) -> Dict[str, List[str]]:
        """Trouver les voisins en utilisant la triangulation de Delaunay, avec filtrage sur le rayon du cercle circonscrit et adjacence"""
        neighbors = {robot: [] for robot in positions.keys()}
        
        if len(positions) < 3:
            self.get_logger().warning("Pas assez de robots pour la triangulation de Delaunay")
            return neighbors
        
        robot_names = list(positions.keys())
        points = np.array([positions[name] for name in robot_names])
        tri = Delaunay(points)

        total_edges = 0
        filtered_by_distance = 0
        filtered_by_alignment = 0
        filtered_by_circumcircle = 0
        
        # Collecter toutes les connexions potentielles depuis Delaunay
        potential_connections = {}  # robot -> liste des (voisin, distance)
        for robot in robot_names:
            potential_connections[robot] = []

        # Fonction pour calculer le centre et le rayon du cercle circonscrit
        def circumcircle(A, B, C):
            D = 2 * (A[0]*(B[1]-C[1]) + B[0]*(C[1]-A[1]) + C[0]*(A[1]-B[1]))
            if D == 0:
                return None, None
            Ux = ((np.linalg.norm(A)**2)*(B[1]-C[1]) + (np.linalg.norm(B)**2)*(C[1]-A[1]) + (np.linalg.norm(C)**2)*(A[1]-B[1])) / D
            Uy = ((np.linalg.norm(A)**2)*(C[0]-B[0]) + (np.linalg.norm(B)**2)*(A[0]-C[0]) + (np.linalg.norm(C)**2)*(B[0]-A[0])) / D
            center = np.array([Ux, Uy])
            radius = np.linalg.norm(center - A)
            return center, radius

        # Pour chaque triangle de la triangulation
        for simplex in tri.simplices:
            pts = points[simplex]
            center, radius = circumcircle(pts[0], pts[1], pts[2])
            # Filtrer par rayon du cercle circonscrit
            if center is None or radius is None or radius > self.max_circumcircle_radius:
                filtered_by_circumcircle += 1
                continue

            # Pour chaque arête du triangle, collecter les connexions potentielles
            for i in range(3):
                for j in range(i + 1, 3):
                    robot1 = robot_names[simplex[i]]
                    robot2 = robot_names[simplex[j]]
                    total_edges += 1

                    distance = self.calculate_distance(positions[robot1], positions[robot2])
                    if distance > self.neighbor_distance_threshold:
                        filtered_by_distance += 1
                        continue

                    if not self.allow_diagonal_neighbors:
                        if not self.is_aligned(positions[robot1], positions[robot2]):
                            filtered_by_alignment += 1
                            continue

                    # Ajouter aux connexions potentielles
                    potential_connections[robot1].append((robot2, distance))
                    potential_connections[robot2].append((robot1, distance))

        # Maintenant, pour chaque robot, ne garder que les voisins adjacents les plus proches
        accepted_edges = 0
        if not self.allow_diagonal_neighbors:
            for robot_name, position in positions.items():
                candidates = potential_connections[robot_name]
                
                if not candidates:
                    continue
                
                # Organiser par direction
                horizontal_candidates = []
                vertical_candidates = []
                
                for candidate_robot, distance in candidates:
                    candidate_position = positions[candidate_robot]
                    dx = abs(position[0] - candidate_position[0])
                    dy = abs(position[1] - candidate_position[1])
                    
                    if dy < self.alignment_tolerance:  # Alignement horizontal
                        direction = 'left' if candidate_position[0] < position[0] else 'right'
                        horizontal_candidates.append((candidate_robot, distance, direction))
                    elif dx < self.alignment_tolerance:  # Alignement vertical
                        direction = 'down' if candidate_position[1] < position[1] else 'up'
                        vertical_candidates.append((candidate_robot, distance, direction))
                
                # Sélectionner le plus proche dans chaque direction
                selected_neighbors = []
                
                # Horizontal
                if horizontal_candidates:
                    left_candidates = [(robot, dist, dir) for robot, dist, dir in horizontal_candidates if dir == 'left']
                    right_candidates = [(robot, dist, dir) for robot, dist, dir in horizontal_candidates if dir == 'right']
                    
                    if left_candidates:
                        closest_left = min(left_candidates, key=lambda x: x[1])
                        selected_neighbors.append(closest_left[0])
                    
                    if right_candidates:
                        closest_right = min(right_candidates, key=lambda x: x[1])
                        selected_neighbors.append(closest_right[0])
                
                # Vertical
                if vertical_candidates:
                    down_candidates = [(robot, dist, dir) for robot, dist, dir in vertical_candidates if dir == 'down']
                    up_candidates = [(robot, dist, dir) for robot, dist, dir in vertical_candidates if dir == 'up']
                    
                    if down_candidates:
                        closest_down = min(down_candidates, key=lambda x: x[1])
                        selected_neighbors.append(closest_down[0])
                    
                    if up_candidates:
                        closest_up = min(up_candidates, key=lambda x: x[1])
                        selected_neighbors.append(closest_up[0])
                
                # Ajouter les connexions sélectionnées
                for neighbor_robot in selected_neighbors:
                    if neighbor_robot not in neighbors[robot_name]:
                        neighbors[robot_name].append(neighbor_robot)
                        accepted_edges += 1
                        distance = self.calculate_distance(position, positions[neighbor_robot])
                        self.get_logger().debug(f"Connexion {robot_name}-{neighbor_robot} acceptée: distance={distance:.3f}m")
        else:
            # Mode avec voisins diagonaux - utiliser toutes les connexions potentielles
            for robot_name in positions.keys():
                for candidate_robot, distance in potential_connections[robot_name]:
                    if candidate_robot not in neighbors[robot_name]:
                        neighbors[robot_name].append(candidate_robot)
                        accepted_edges += 1

        self.get_logger().info(f"Filtrage Delaunay: {total_edges} arêtes totales -> {accepted_edges} acceptées")
        self.get_logger().info(f"Rejetées: {filtered_by_distance} par distance, {filtered_by_alignment} par alignement, {filtered_by_circumcircle} triangles par rayon de cercle > {self.max_circumcircle_radius}m")
        return neighbors

    def plot_formation_graph(self, positions: Dict[str, Tuple[float, float]], 
                           neighbors: Dict[str, List[str]]):
        """Tracer le graphe de formation et les cercles circonscrits de Delaunay (filtrés par rayon)"""
        if not positions:
            return
            
        if self.figure is None:
            self.figure = plt.figure(figsize=(10, 8))
        else:
            plt.figure(self.figure.number)
            plt.clf()
        
        # Tracer les robots avec axes inversés (Y vers la gauche, X vers le haut)
        x_coords = []
        y_coords = []
        labels = []
        
        for robot_name, (x, y) in positions.items():
            x_coords.append(y)  # Y original devient -Y (vers la gauche)
            y_coords.append(x)   # X original devient Y (vers le haut)
            labels.append(robot_name)
        
        plt.scatter(x_coords, y_coords, s=100, c='blue', marker='o', zorder=5)
        
        # Ajouter les annotations
        for i, label in enumerate(labels):
            plt.annotate(label, (x_coords[i], y_coords[i]), 
                        xytext=(5, 5), textcoords='offset points', 
                        fontsize=10, fontweight='bold')
        
        # Tracer les connexions
        connection_count = 0
        for robot1, neighbors_list in neighbors.items():
            if robot1 not in positions:
                continue
            pos1 = positions[robot1]
            pos1_transformed = (-pos1[1], pos1[0])
            
            for robot2 in neighbors_list:
                if robot2 not in positions:
                    continue
                pos2 = positions[robot2]
                pos2_transformed = (-pos2[1], pos2[0])
                
                if robot1 < robot2:  # Éviter les doublons
                    plt.plot([-pos1_transformed[0], -pos2_transformed[0]], 
                            [pos1_transformed[1], pos2_transformed[1]], 
                            'r-', alpha=0.8, linewidth=2, zorder=3)
                    connection_count += 1

        # === Ajout du tracé des cercles circonscrits de Delaunay (filtrés par rayon) ===
        if len(positions) >= 3:
            robot_names = list(positions.keys())
            points = np.array([positions[name] for name in robot_names])
            try:
                tri = Delaunay(points)
                for simplex in tri.simplices:
                    pts = points[simplex]
                    A = pts[0]
                    B = pts[1]
                    C = pts[2]
                    def circumcircle(A, B, C):
                        D = 2 * (A[0]*(B[1]-C[1]) + B[0]*(C[1]-A[1]) + C[0]*(A[1]-B[1]))
                        if D == 0:
                            return None, None
                        Ux = ((np.linalg.norm(A)**2)*(B[1]-C[1]) + (np.linalg.norm(B)**2)*(C[1]-A[1]) + (np.linalg.norm(C)**2)*(A[1]-B[1])) / D
                        Uy = ((np.linalg.norm(A)**2)*(C[0]-B[0]) + (np.linalg.norm(B)**2)*(A[0]-C[0]) + (np.linalg.norm(C)**2)*(B[0]-A[0])) / D
                        center = np.array([Ux, Uy])
                        radius = np.linalg.norm(center - A)
                        return center, radius
                    center, radius = circumcircle(A, B, C)
                    # Ne tracer le cercle que si son rayon est inférieur ou égal au seuil
                    if center is not None and radius is not None and radius <= self.max_circumcircle_radius:
                        cx, cy = center[1], center[0]
                        circle = plt.Circle((cx, cy), radius, color='g', fill=False, linestyle='--', linewidth=1.5, alpha=0.5, zorder=2)
                        plt.gca().add_patch(circle)
            except Exception as e:
                self.get_logger().error(f"Erreur lors du tracé des cercles circonscrits: {e}")

        plt.title('Formation - Triangulation de Delaunay', fontsize=14, fontweight='bold')
        plt.xlabel('Position Y (m) ', fontsize=12)
        plt.ylabel('Position X (m) ', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.gca().invert_xaxis()
        
        plt.draw()
        plt.pause(0.1)
        
        self.get_logger().info(f"Tracé {len(positions)} robots avec {connection_count} connexions")

    def write_neighbors_to_yaml(self, neighbors: Dict[str, List[str]]):
        """Écrire les relations de voisinage dans le fichier YAML"""
        if not self.write_to_yaml:
            return
            
        try:
            # Résoudre le chemin complet
            yaml_path = os.path.expanduser(self.yaml_file_path)
            
            # Si le chemin est relatif, le baser sur le répertoire du package
            if not os.path.isabs(yaml_path):
                try:
                    package_share_dir = get_package_share_directory('mecanum_swarm')
                    yaml_path = os.path.join(package_share_dir, yaml_path)
                except:
                    # Fallback vers le répertoire courant
                    yaml_path = os.path.abspath(yaml_path)
            
            # Lire le fichier YAML existant ou créer une structure de base
            yaml_data = {}
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as file:
                    yaml_data = yaml.safe_load(file) or {}
            
            # Mettre à jour les relations de voisinage
            yaml_data['robot_neighbors'] = {robot: neighbors_list for robot, neighbors_list in neighbors.items() if neighbors_list}
            
            # Maintenir la liste des robots si elle existe
            if 'all_robot_names' not in yaml_data:
                yaml_data['all_robot_names'] = list(neighbors.keys())
            
            # Écrire dans le fichier YAML
            os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
            with open(yaml_path, 'w') as file:
                yaml.dump(yaml_data, file, default_flow_style=False, sort_keys=True)
            
            self.get_logger().info(f"Relations de voisinage écrites dans {yaml_path}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture du fichier YAML: {e}")

    def update_formation(self):
        """Mettre à jour le graphe de formation"""
        positions = self.get_robot_positions()
        
        if len(positions) < 1:
            self.get_logger().debug("Aucun robot détecté")
            return
            
        self.get_logger().info(f"Robots détectés: {list(positions.keys())}")
        
        # Effacer le graphe précédent
        self.formation_graph = FormationGraph()
        
        # Ajouter tous les robots au graphe
        for robot_name, position in positions.items():
            self.formation_graph.add_robot(robot_name, position)
        
        # Trouver les voisins avec Delaunay
        neighbors = self.find_delaunay_neighbors(positions)
        
        # Ajouter les relations de voisinage au graphe
        for robot, robot_neighbors in neighbors.items():
            for neighbor in robot_neighbors:
                self.formation_graph.add_neighbor_relationship(robot, neighbor)
        
        # Écrire les relations dans le fichier YAML
        self.write_neighbors_to_yaml(neighbors)
        
        # Afficher les informations
        self.get_logger().info(f"Formation mise à jour avec {len(positions)} robots")
        self.formation_graph.print_formation()
        
        # Tracer le graphe si activé
        if self.enable_plotting:
            try:
                self.plot_formation_graph(positions, neighbors)
            except Exception as e:
                self.get_logger().error(f"Erreur lors du traçage: {e}")
    
    def get_formation_graph(self) -> FormationGraph:
        """Obtenir le graphe de formation actuel"""
        return self.formation_graph
    
    def get_robot_neighbors(self, robot_name: str) -> List[str]:
        """Obtenir les voisins d'un robot spécifique"""
        return self.formation_graph.get_neighbors(robot_name)

def main(args=None):
    rclpy.init(args=args)
    node = FormationInitializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
