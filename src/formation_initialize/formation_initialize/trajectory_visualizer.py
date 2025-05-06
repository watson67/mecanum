#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import matplotlib
matplotlib.use('TkAgg')  
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import math
import time
import threading
from matplotlib.animation import FuncAnimation

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # Configuration
        self.robot_names = ["Aramis", "Athos", "Porthos"]
        self.colors = ['red', 'green', 'blue']  # Couleurs pour chaque robot
        self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
        
        # Configuration du QoS profile pour MOCAP
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Stockage des positions
        self.trajectories = {name: {'x': [], 'y': [], 'time': []} for name in self.robot_names}
        self.current_positions = {name: None for name in self.robot_names}
        
        # Stockage des trajectoires planifiées
        self.planned_trajectories = {name: {'x': [], 'y': []} for name in self.robot_names}
        self.has_planned_trajectories = False
        
        # Souscriptions aux poses
        for i, topic in enumerate(self.pose_topics):
            self.create_subscription(
                PoseStamped, 
                topic, 
                self.pose_callback_factory(self.robot_names[i]),
                qos_profile=self.qos_profile
            )
        
        # Souscription aux trajectoires planifiées
        self.create_subscription(
            Float32MultiArray,
            '/formation_initializer/planned_trajectories',
            self.planned_trajectory_callback,
            10
        )
        
        # Flag pour savoir si le graphique est prêt
        self.plot_initialized = False
        
        # Timer pour la mise à jour du graphique
        self.update_interval = 0.5  # secondes
        self.timer = self.create_timer(self.update_interval, self.update_plot)
        
        # Paramètres pour l'animation
        self.start_time = None
        self.fig = None
        self.ax = None
        
        # Démarrer le thread d'affichage séparé
        self.plot_thread = threading.Thread(target=self.initialize_plot)
        self.plot_thread.daemon = True  # Le thread s'arrêtera quand le programme principal s'arrête
        self.plot_thread.start()
        
        self.get_logger().info('Trajectory visualizer started')

    def initialize_plot(self):
        """Initialise l'affichage dans un thread séparé"""
        # Créer une figure avec gridspec pour un meilleur contrôle des proportions
        self.fig = plt.figure(figsize=(14, 8))
        grid = plt.GridSpec(1, 4, figure=self.fig)  # 1 ligne, 4 colonnes pour un ratio 3:1
        
        # Graphique de trajectoire à gauche (75% de l'espace - 3 colonnes sur 4)
        self.ax = self.fig.add_subplot(grid[0, 0:3])
        
        # Zone de texte à droite (25% de l'espace - 1 colonne sur 4)
        self.text_ax = self.fig.add_subplot(grid[0, 3])
        self.text_ax.axis('off')  # Pas d'axes pour la zone de texte
        
        # Ajouter un espace pour le bouton
        button_ax = plt.axes([0.35, 0.05, 0.15, 0.05])
        self.clear_button = Button(button_ax, 'Clear Trajectories')
        self.clear_button.on_clicked(self.clear_trajectories)
        
        self.setup_plot()
        self.plot_initialized = True
        self.get_logger().info('Plot window initialized')
        plt.show()  # Cette commande est bloquante

    def clear_trajectories(self, event=None):
        """Efface toutes les trajectoires affichées"""
        if not self.plot_initialized:
            return
            
        # Réinitialiser les trajectoires
        for name in self.robot_names:
            self.trajectories[name] = {'x': [], 'y': [], 'time': []}
            self.trajectory_lines[name].set_data([], [])
        
        # Ne pas effacer les trajectoires planifiées car elles sont importantes
        
        self.get_logger().info('Trajectories cleared')
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def pose_callback_factory(self, name):
        def callback(msg):
            # Obtenir le temps actuel
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Si c'est le premier message, initialiser start_time
            if self.start_time is None:
                self.start_time = current_time
            
            # Calculer le temps relatif
            relative_time = current_time - self.start_time
            
            # Enregistrer la position
            position = (msg.pose.position.x, msg.pose.position.y)
            self.current_positions[name] = position
            
            # Ajouter à la trajectoire
            self.trajectories[name]['x'].append(position[0])
            self.trajectories[name]['y'].append(position[1])
            self.trajectories[name]['time'].append(relative_time)
            
            # Imprimer la position pour vérifier que les données sont reçues
            self.get_logger().info(f'{name} position: ({position[0]:.2f}, {position[1]:.2f})')
            
        return callback

    def setup_plot(self):
        """Configure le graphique initial"""
        self.ax.set_ylabel('X Position (m)')  # X sur l'axe Y
        self.ax.set_xlabel('Y Position (m)')  # Y sur l'axe X
        self.ax.set_title('Robot Trajectories')
        
        # Définir une grille
        self.ax.grid(True)
        
        # Mise à l'échelle avec un ratio égal
        self.ax.set_aspect('equal')
        
        # Définir des limites fixes
        self.ax.set_xlim(2, -2)  # Inverser pour avoir y vers la gauche
        self.ax.set_ylim(-2, 2)
        
        # Créer les lignes et points pour chaque robot
        self.trajectory_lines = {}
        self.position_markers = {}
        self.planned_trajectory_lines = {}  # Nouvelles lignes pour les trajectoires planifiées
        
        for i, name in enumerate(self.robot_names):
            color = self.colors[i % len(self.colors)]
            
            # Ligne pour la trajectoire réelle
            line, = self.ax.plot([], [], '-', color=color, label=f'{name} actual', linewidth=2, alpha=0.7)
            self.trajectory_lines[name] = line
            
            # Ligne pour la trajectoire planifiée (pointillée)
            planned_line, = self.ax.plot([], [], '--', color=color, label=f'{name} planned', linewidth=1.5, alpha=0.9)
            self.planned_trajectory_lines[name] = planned_line
            
            # Marqueur pour la position actuelle
            marker, = self.ax.plot([], [], 'o', color=color, markersize=10)
            self.position_markers[name] = marker
        
        # Ne pas ajouter la légende ici, elle sera affichée dans le texte
        
        # Définir les limites initiales
        self.ax.set_xlim(1, -2)  # Inverser pour avoir y vers la gauche
        self.ax.set_ylim(-2, 2)
        
        # Initialiser la zone de texte pour les positions
        self.position_text = self.text_ax.text(0.05, 0.95, 'Robot positions:',
                                              va='top', ha='left',
                                              transform=self.text_ax.transAxes,
                                              fontsize=10)

    def planned_trajectory_callback(self, msg):
        """Callback pour recevoir les trajectoires planifiées"""
        try:
            # Décoder le message contenant toutes les trajectoires planifiées
            data = msg.data
            if not data:
                return
                
            # Format: [robot_count, r1_name_len, r1_name_chars..., r1_waypoint_count, r1_x1, r1_y1, r1_x2, r1_y2, ..., r2_name_len, ...]
            idx = 0
            robot_count = int(data[idx])
            idx += 1
            
            self.get_logger().info(f"Received planned trajectories for {robot_count} robots")
            
            # Réinitialiser les trajectoires planifiées
            self.planned_trajectories = {name: {'x': [], 'y': []} for name in self.robot_names}
            
            # Décoder les trajectoires pour chaque robot
            for _ in range(robot_count):
                # Décoder le nom du robot
                name_len = int(data[idx])
                idx += 1
                name_chars = [chr(int(data[idx + i])) for i in range(name_len)]
                idx += name_len
                name = ''.join(name_chars)
                
                # Décoder les waypoints
                waypoint_count = int(data[idx])
                idx += 1
                
                x_coords = []
                y_coords = []
                
                for _ in range(waypoint_count):
                    x = float(data[idx])
                    idx += 1
                    y = float(data[idx])
                    idx += 1
                    x_coords.append(x)
                    y_coords.append(y)
                
                if name in self.planned_trajectories:
                    self.planned_trajectories[name]['x'] = x_coords
                    self.planned_trajectories[name]['y'] = y_coords
                    self.get_logger().info(f"Decoded trajectory for {name} with {waypoint_count} waypoints")
            
            self.has_planned_trajectories = True
            
        except Exception as e:
            self.get_logger().error(f"Error decoding planned trajectories: {e}")

    def update_plot(self):
        """Met à jour le graphique avec les positions actuelles et les trajectoires planifiées"""
        if not self.plot_initialized:
            return
            
        try:
            # Mise à jour des trajectoires réelles
            for name in self.robot_names:
                # Obtenir les données de trajectoire réelle
                x_data = self.trajectories[name]['x']
                y_data = self.trajectories[name]['y']
                
                # Mettre à jour la ligne de trajectoire réelle (inverser x et y pour le plot)
                if x_data and y_data:
                    self.trajectory_lines[name].set_data(y_data, x_data)
                    
                    # Mettre à jour le marqueur de position actuelle
                    if self.current_positions[name]:
                        self.position_markers[name].set_data([self.current_positions[name][1]], 
                                                            [self.current_positions[name][0]])
            
            # Mise à jour des trajectoires planifiées
            if self.has_planned_trajectories:
                for name in self.robot_names:
                    # Obtenir les données de trajectoire planifiée
                    planned_x = self.planned_trajectories[name]['x']
                    planned_y = self.planned_trajectories[name]['y']
                    
                    # Mettre à jour la ligne de trajectoire planifiée (inverser x et y pour le plot)
                    if planned_x and planned_y:
                        self.planned_trajectory_lines[name].set_data(planned_y, planned_x)
            
            # Ajuster les limites si nécessaire
            self.adjust_plot_limits()
            
            # Mettre à jour les informations de position dans la zone de texte
            self.update_position_info()
            
            # Forcer le rafraîchissement du graphique
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f'Error updating plot: {e}')
    
    def update_position_info(self):
        """Met à jour les informations de position et distances dans la zone de texte"""
        if not all(self.current_positions.values()):
            return  # Attendre que toutes les positions soient disponibles
        
        # Construction du texte avec la légende
        info_text = "Legend:\n\n"
        
        # Ajouter la légende personnalisée avec les noms de couleurs explicites
        for i, name in enumerate(self.robot_names):
            color_name = self.colors[i % len(self.colors)]
            
            # Utiliser des caractères unicode pour créer des lignes colorées dans le texte
            info_text += f"{name:7} ({color_name}):     ——————  (actual)\n"
            info_text += f"{' ':7}            - - - -  (planned)\n\n"
        
        info_text += "Robot positions:\n\n"
        
        # Format aligné pour les positions
        position_format = "{name:8}: X={x:8.3f} m, Y={y:8.3f} m\n"
        
        for i, name in enumerate(self.robot_names):
            if self.current_positions[name]:
                x, y = self.current_positions[name]
                info_text += position_format.format(name=name, x=x, y=y)
        
        info_text += "\nDistances between robots:\n\n"
        
        # Format aligné pour les distances
        distance_format = "{name1:8} - {name2:8}: {distance:8.3f} m\n"
        
        # Calculer les distances entre les robots
        for i, name1 in enumerate(self.robot_names):
            for j, name2 in enumerate(self.robot_names):
                if i < j:  # Pour éviter les doublons
                    if self.current_positions[name1] and self.current_positions[name2]:
                        x1, y1 = self.current_positions[name1]
                        x2, y2 = self.current_positions[name2]
                        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        info_text += distance_format.format(name1=name1, name2=name2, distance=distance)
        
        # Configurer le texte
        self.position_text.set_text(info_text)
        self.position_text.set_fontfamily('monospace')

        # Ajouter une bordure autour du texte pour le rendre plus lisible
        plt.setp(self.position_text, bbox=dict(facecolor='white', alpha=0.8, pad=10))
        self.text_ax.figure.canvas.draw_idle()
    
    def adjust_plot_limits(self):
        """Maintient les limites fixes du graphique"""
        # Toujours maintenir les limites fixes comme demandé
        self.ax.set_xlim(2, -2)  # Y sur l'axe X (inversé)
        self.ax.set_ylim(-2, 2)  # X sur l'axe Y

    # The save_trajectories method has been removed

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Removed the trajectory saving call
        pass
    finally:
        # Nettoyage
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
