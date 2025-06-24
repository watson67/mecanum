import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

class FormationInitializer(Node):
    def __init__(self):
        super().__init__('formation_initializer')

        self.robot_names = ["Aramis", "Athos", "Porthos"]

        # Topics
        self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
        self.cmd_vel_topics = [f"/{name}/cmd_vel" for name in self.robot_names]

        # Publishers pour les vitesses
        self.cmd_publishers = []
        for topic in self.cmd_vel_topics:
            pub = self.create_publisher(Twist, topic, 10)
            self.cmd_publishers.append(pub)

        # Souscriptions aux poses
        self.positions = {name: None for name in self.robot_names}
        for i, topic in enumerate(self.pose_topics):
            self.create_subscription(PoseStamped, topic, self.pose_callback_factory(self.robot_names[i]), 10)

        # Positions cibles
        self.targets = {
            "Aramis": (1, 2),
            "Athos": (1, 4),
            "Porthos": (1, 1)
        }

        self.timer = self.create_timer(0.1, self.control_loop)

        # Variable pour suivre l'état de la formation
        self.formation_complete = False

        # Paramètres pour évitement de collision
        self.safety_distance = 0.3  # Distance minimale entre robots (en mètres)
        self.collision_gain = 1.2   # Gain pour la force répulsive
        self.detection_radius = 1.0 # Rayon de détection des collisions

    def pose_callback_factory(self, name):
        def callback(msg):
            self.positions[name] = (msg.pose.position.x, msg.pose.position.y)
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

    def calculate_repulsive_force(self, robot_name, robot_pos):
        """Calcule les forces répulsives pour éviter les collisions entre robots"""
        repulsive_fx = 0.0
        repulsive_fy = 0.0
        collision_detected = False
        
        # Vérifier les distances avec les autres robots
        for other_name, other_pos in self.positions.items():
            if other_name == robot_name or other_pos is None:
                continue
                
            dx = robot_pos[0] - other_pos[0]
            dy = robot_pos[1] - other_pos[1]
            distance = math.hypot(dx, dy)
            
            # Si les robots sont trop proches, appliquer une force répulsive
            if distance < self.detection_radius:
                collision_detected = True
                # Plus les robots sont proches, plus la force est forte
                magnitude = self.collision_gain * (self.safety_distance / (distance + 0.01) - 1.0)
                if magnitude > 0:  # Force répulsive seulement si plus proche que safety_distance
                    # Normaliser le vecteur direction
                    if distance > 0.01:  # Éviter division par zéro
                        repulsive_fx += magnitude * (dx / distance)
                        repulsive_fy += magnitude * (dy / distance)
        
        return repulsive_fx, repulsive_fy, collision_detected

    def control_loop(self):
        all_robots_on_target = True
        
        for i, name in enumerate(self.robot_names):
            position = self.positions[name]
            if position is None:
                all_robots_on_target = False
                continue  # On n'a pas encore la pose de ce robot

            target = self.targets[name]
            error_x = target[0] - position[0]
            error_y = target[1] - position[1]

            distance = math.hypot(error_x, error_y)
            cmd = Twist()

            if distance > 0.05:  # tolérance de 5cm
                all_robots_on_target = False
                
                # Contrôle proportionnel vers la cible
                k_p = 0.5  
                target_vx = k_p * error_x
                target_vy = k_p * error_y
                
                # Calcul des forces répulsives pour éviter les collisions
                repulsive_fx, repulsive_fy, collision_risk = self.calculate_repulsive_force(name, position)
                
                # Appliquer les forces répulsives à la commande
                cmd.linear.x = target_vx + repulsive_fx
                cmd.linear.y = target_vy + repulsive_fy
                
                # Limiter la vitesse si nécessaire
                speed = math.hypot(cmd.linear.x, cmd.linear.y)
                max_speed = 1.0
                if speed > max_speed:
                    cmd.linear.x = (cmd.linear.x / speed) * max_speed
                    cmd.linear.y = (cmd.linear.y / speed) * max_speed
                
                cmd.linear.z = 0.0
                cmd.angular.z = 0.0
                
                # Afficher des informations supplémentaires si risque de collision
                if collision_risk:
                    self.get_logger().info(f"Collision risk for {name}! Applying avoidance: fx={repulsive_fx:.2f}, fy={repulsive_fy:.2f}")
            else:
                # Si on est arrivé, on arrête les robots
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

            self.cmd_publishers[i].publish(cmd)
            self.get_logger().info(f"Sending cmd to {name}: vx={cmd.linear.x:.2f} vy={cmd.linear.y:.2f}")
        
        # Vérifie si tous les robots ont atteint leurs cibles
        if all_robots_on_target:
            self.get_logger().info("Formation complete! Shutting down...")
            self.formation_complete = True
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FormationInitializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
