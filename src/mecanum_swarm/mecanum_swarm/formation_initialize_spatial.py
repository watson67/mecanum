import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import numpy as np
from typing import Dict, List, Tuple, Set
from mecanum_swarm.config import ALL_ROBOT_NAMES
import matplotlib.pyplot as plt
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SpatialGrid:
    """Grille spatiale pour la recherche optimisée de voisins"""
    
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.grid = {}  # (grid_x, grid_y) -> liste des robots dans cette cellule
        self.robot_positions = {}  # robot_name -> (x, y)
    
    def _get_cell_coords(self, x: float, y: float) -> Tuple[int, int]:
        """Convertir les coordonnées world en coordonnées de cellule"""
        return (int(x // self.cell_size), int(y // self.cell_size))
    
    def clear(self):
        """Vider la grille"""
        self.grid.clear()
        self.robot_positions.clear()
    
    def add_robot(self, robot_name: str, x: float, y: float):
        """Ajouter un robot à la grille"""
        cell_coords = self._get_cell_coords(x, y)
        
        if cell_coords not in self.grid:
            self.grid[cell_coords] = []
        
        self.grid[cell_coords].append(robot_name)
        self.robot_positions[robot_name] = (x, y)
    
    def get_nearby_robots(self, robot_name: str, radius: float) -> List[str]:
        """Obtenir tous les robots dans un rayon donné en utilisant la grille spatiale"""
        if robot_name not in self.robot_positions:
            return []
        
        x, y = self.robot_positions[robot_name]
        nearby_robots = []
        
        # Calculer les cellules à vérifier (carré autour de la position)
        cell_radius = int(math.ceil(radius / self.cell_size))
        center_cell = self._get_cell_coords(x, y)
        
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                check_cell = (center_cell[0] + dx, center_cell[1] + dy)
                
                if check_cell in self.grid:
                    for other_robot in self.grid[check_cell]:
                        if other_robot != robot_name:
                            other_x, other_y = self.robot_positions[other_robot]
                            distance = math.sqrt((x - other_x)**2 + (y - other_y)**2)
                            
                            if distance <= radius:
                                nearby_robots.append(other_robot)
        
        return nearby_robots

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
        print("\n=== Graphe de Formation (Spatial Hashing) ===")
        for robot, neighbors in self.neighbors.items():
            pos = self.nodes[robot]
            print(f"{robot} à ({pos[0]:.2f}, {pos[1]:.2f}) -> voisins: {neighbors}")
        print(f"Total d'arêtes: {len(self.edges)}")

class FormationInitializer(Node):
    def __init__(self):
        super().__init__('formation_initializer_spatial')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.formation_graph = FormationGraph()
        
        # Paramètres pour la recherche spatiale
        self.neighbor_distance_threshold = self.declare_parameter('neighbor_distance_threshold', 0.8).value  # mètres
        self.grid_cell_size = self.declare_parameter('grid_cell_size', 0.5).value  # taille des cellules de la grille
        
        # Paramètres d'alignement
        self.alignment_tolerance = self.declare_parameter('alignment_tolerance', 0.1).value  # mètres
        self.allow_diagonal_neighbors = self.declare_parameter('allow_diagonal_neighbors', False).value
        
        # Paramètres de visualisation
        self.enable_plotting = self.declare_parameter('enable_plotting', False).value
        
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
        
        # Initialiser la grille spatiale
        self.spatial_grid = SpatialGrid(self.grid_cell_size)
        
        # Variables pour le traçage
        self.figure = None
        
        # Timer pour mettre à jour la formation
        self.create_timer(1.0, self.update_formation)  # 1 Hz
        
        self.get_logger().info("Initialisateur de Formation démarré - Méthode: Fixed-radius + Spatial Hashing")
        self.get_logger().info(f"Seuil de distance: {self.neighbor_distance_threshold}m")
        self.get_logger().info(f"Taille des cellules de grille: {self.grid_cell_size}m")
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
    
    def find_spatial_neighbors(self, positions: Dict[str, Tuple[float, float]]) -> Dict[str, List[str]]:
        """
        Trouver les voisins en utilisant la recherche spatiale avec grille.
        Cette méthode garantit que seuls les voisins adjacents les plus proches sont connectés,
        évitant les connexions qui "sautent" par-dessus d'autres robots.
        
        Args:
            positions: Dictionnaire {nom_robot: (x, y)} des positions
            
        Returns:
            Dictionnaire {nom_robot: [liste_voisins]} des relations de voisinage
        """
        neighbors = {robot: [] for robot in positions.keys()}
        
        if len(positions) < 2:
            self.get_logger().warning("Pas assez de robots pour la recherche de voisins")
            return neighbors
        
        # Vider et remplir la grille spatiale pour optimiser la recherche
        self.spatial_grid.clear()
        for robot_name, (x, y) in positions.items():
            self.spatial_grid.add_robot(robot_name, x, y)
        
        # Statistiques pour le debugging
        total_checks = 0
        filtered_by_distance = 0
        filtered_by_alignment = 0
        filtered_by_adjacency = 0
        accepted_connections = 0
        
        # Pour chaque robot, chercher ses voisins dans chaque direction cardinale
        for robot_name, position in positions.items():
            # Utiliser la grille spatiale pour obtenir les candidats dans le rayon
            nearby_robots = self.spatial_grid.get_nearby_robots(robot_name, self.neighbor_distance_threshold)
            
            if not self.allow_diagonal_neighbors:
                # === MODE ALIGNEMENT STRICT (pas de diagonales) ===
                # Organiser les candidats par direction (horizontal/vertical uniquement)
                horizontal_neighbors = []  # robots alignés horizontalement (même Y)
                vertical_neighbors = []    # robots alignés verticalement (même X)
                
                for nearby_robot in nearby_robots:
                    total_checks += 1
                    nearby_position = positions[nearby_robot]
                    distance = self.calculate_distance(position, nearby_position)
                    
                    # Première vérification: distance
                    if distance > self.neighbor_distance_threshold:
                        filtered_by_distance += 1
                        continue
                    
                    # Calculer les différences de coordonnées pour déterminer l'alignement
                    dx = abs(position[0] - nearby_position[0])
                    dy = abs(position[1] - nearby_position[1])
                    
                    is_horizontally_aligned = dy < self.alignment_tolerance
                    is_vertically_aligned = dx < self.alignment_tolerance
                    
                    if is_horizontally_aligned:
                        # Robots alignés horizontalement - déterminer la direction
                        direction = 'left' if nearby_position[0] < position[0] else 'right'
                        horizontal_neighbors.append((nearby_robot, distance, direction, nearby_position[0]))
                    elif is_vertically_aligned:
                        # Robots alignés verticalement - déterminer la direction
                        direction = 'down' if nearby_position[1] < position[1] else 'up'
                        vertical_neighbors.append((nearby_robot, distance, direction, nearby_position[1]))
                    else:
                        # Robot ni aligné horizontalement ni verticalement
                        filtered_by_alignment += 1
                
                # === SÉLECTION DU VOISIN LE PLUS PROCHE DANS CHAQUE DIRECTION ===
                closest_neighbors = []
                
                # Traitement des voisins horizontaux (même ligne Y)
                if horizontal_neighbors:
                    # Séparer les voisins à gauche et à droite
                    left_neighbors = [(robot, dist, dir, coord) for robot, dist, dir, coord in horizontal_neighbors if dir == 'left']
                    right_neighbors = [(robot, dist, dir, coord) for robot, dist, dir, coord in horizontal_neighbors if dir == 'right']
                    
                    # Prendre le voisin le plus proche à gauche
                    if left_neighbors:
                        closest_left = min(left_neighbors, key=lambda x: x[1])  # x[1] = distance
                        closest_neighbors.append(closest_left[0])
                        self.get_logger().debug(f"{robot_name}: voisin gauche le plus proche: {closest_left[0]} à {closest_left[1]:.3f}m")
                    
                    # Prendre le voisin le plus proche à droite
                    if right_neighbors:
                        closest_right = min(right_neighbors, key=lambda x: x[1])
                        closest_neighbors.append(closest_right[0])
                        self.get_logger().debug(f"{robot_name}: voisin droite le plus proche: {closest_right[0]} à {closest_right[1]:.3f}m")
                
                # Traitement des voisins verticaux (même colonne X)
                if vertical_neighbors:
                    # Séparer les voisins en haut et en bas
                    down_neighbors = [(robot, dist, dir, coord) for robot, dist, dir, coord in vertical_neighbors if dir == 'down']
                    up_neighbors = [(robot, dist, dir, coord) for robot, dist, dir, coord in vertical_neighbors if dir == 'up']
                    
                    # Prendre le voisin le plus proche en bas
                    if down_neighbors:
                        closest_down = min(down_neighbors, key=lambda x: x[1])
                        closest_neighbors.append(closest_down[0])
                        self.get_logger().debug(f"{robot_name}: voisin bas le plus proche: {closest_down[0]} à {closest_down[1]:.3f}m")
                    
                    # Prendre le voisin le plus proche en haut
                    if up_neighbors:
                        closest_up = min(up_neighbors, key=lambda x: x[1])
                        closest_neighbors.append(closest_up[0])
                        self.get_logger().debug(f"{robot_name}: voisin haut le plus proche: {closest_up[0]} à {closest_up[1]:.3f}m")
                
                # Ajouter les connexions sélectionnées (unidirectionnelles pour éviter les doublons)
                for neighbor_robot in closest_neighbors:
                    if neighbor_robot not in neighbors[robot_name]:
                        neighbors[robot_name].append(neighbor_robot)
                        accepted_connections += 1
                        distance = self.calculate_distance(position, positions[neighbor_robot])
                        self.get_logger().debug(f"Connexion {robot_name}-{neighbor_robot} acceptée: distance={distance:.3f}m")
            
            else:
                # === MODE AVEC DIAGONALES AUTORISÉES ===
                processed_pairs = set()
                for nearby_robot in nearby_robots:
                    # Éviter les doublons en triant les noms des robots
                    pair = tuple(sorted([robot_name, nearby_robot]))
                    if pair in processed_pairs:
                        continue
                    processed_pairs.add(pair)
                    
                    total_checks += 1
                    distance = self.calculate_distance(position, positions[nearby_robot])
                    
                    if distance > self.neighbor_distance_threshold:
                        filtered_by_distance += 1
                        continue
                    
                    # Ajouter la connexion bidirectionnelle
                    neighbors[robot_name].append(nearby_robot)
                    neighbors[nearby_robot].append(robot_name)
                    accepted_connections += 1
        
        # Calculer les statistiques pour le logging
        total_possible_neighbors = sum(len(self.spatial_grid.get_nearby_robots(robot, self.neighbor_distance_threshold)) for robot in positions.keys())
        filtered_by_adjacency = total_possible_neighbors - total_checks - filtered_by_distance - filtered_by_alignment
        
        # Afficher les statistiques de la recherche
        self.get_logger().info(f"Recherche spatiale: {total_checks} paires vérifiées -> {accepted_connections} connexions acceptées")
        self.get_logger().info(f"Rejetées: {filtered_by_distance} par distance, {filtered_by_alignment} par alignement, {filtered_by_adjacency} par adjacence")
        self.get_logger().info(f"Grille spatiale: {len(self.spatial_grid.grid)} cellules utilisées")
        
        return neighbors
    
    def plot_formation_graph(self, positions: Dict[str, Tuple[float, float]], 
                           neighbors: Dict[str, List[str]]):
        """
        Tracer le graphe de formation avec visualisation simplifiée.
        Affiche uniquement la formation avec les connexions entre voisins.
        
        Args:
            positions: Dictionnaire des positions des robots
            neighbors: Dictionnaire des relations de voisinage
        """
        if not positions:
            return
            
        # Créer ou réutiliser la figure
        if self.figure is None:
            self.figure = plt.figure(figsize=(10, 8))
        else:
            plt.figure(self.figure.number)
            plt.clf()
        
        # === PRÉPARATION DES DONNÉES POUR LE TRACÉ ===
        # Transformation des coordonnées pour l'affichage
        # (axes inversés pour correspondre à la convention robotique)
        x_coords = []
        y_coords = []
        labels = []
        
        for robot_name, (x, y) in positions.items():
            x_coords.append(-y)  # Y original devient -Y (vers la gauche)
            y_coords.append(x)   # X original devient Y (vers le haut)
            labels.append(robot_name)
        
        # === TRACÉ DES ROBOTS ===
        plt.scatter(x_coords, y_coords, s=100, c='blue', marker='o', zorder=5)
        
        # Ajouter les noms des robots comme annotations
        for i, label in enumerate(labels):
            plt.annotate(label, (x_coords[i], y_coords[i]), 
                        xytext=(5, 5), textcoords='offset points', 
                        fontsize=10, fontweight='bold')
        
        # === TRACÉ DES CONNEXIONS ENTRE VOISINS ===
        connection_count = 0
        for robot1, neighbors_list in neighbors.items():
            if robot1 not in positions:
                continue
            pos1 = positions[robot1]
            pos1_transformed = (-pos1[1], pos1[0])  # Transformation pour l'affichage
            
            for robot2 in neighbors_list:
                if robot2 not in positions:
                    continue
                pos2 = positions[robot2]
                pos2_transformed = (-pos2[1], pos2[0])
                
                # Éviter les doublons en ne traçant que si robot1 < robot2 (ordre alphabétique)
                if robot1 < robot2:
                    plt.plot([pos1_transformed[0], pos2_transformed[0]], 
                            [pos1_transformed[1], pos2_transformed[1]], 
                            'r-', alpha=0.8, linewidth=2, zorder=3)
                    connection_count += 1
        
        # === CONFIGURATION DU GRAPHIQUE ===
        plt.title('Formation - Rayon fixe ', fontsize=14, fontweight='bold')
        plt.xlabel('Position Y (m)', fontsize=12)
        plt.ylabel('Position X (m)', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')  # Assurer un ratio d'aspect 1:1
        
        # Finaliser et afficher
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
        
        # Trouver les voisins avec la recherche spatiale
        neighbors = self.find_spatial_neighbors(positions)
        
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
