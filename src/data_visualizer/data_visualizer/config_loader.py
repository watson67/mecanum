import yaml
import os

class ConfigLoader:
    def __init__(self):
        self.robot_names = []
        self.robot_neighbors = {}
        self.robots_with_neighbors = []
        self.pose_topics = []
        self.cmd_vel_topics = []
        self.estimated_position_topics = {}
        self.colors = ['red', 'green', 'blue', 'orange']
    
    def load_robot_config(self):
        """Load robot configuration from YAML file"""
        try:
            # Get the path to the YAML file
            config_path = "/home/eswarm/mecanum/src/swarm_manager/config/robots.yaml"
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            self.robot_names = config['all_robot_names']
            self.robot_neighbors = config['robot_neighbors']
            
            # Create topics
            self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
            self.cmd_vel_topics = [f"/{name}/cmd_vel" for name in self.robot_names]
            
            # Create estimated position topics
            self.estimated_position_topics = {}
            for robot in self.robot_names:
                self.estimated_position_topics[robot] = {}
                if robot in self.robot_neighbors:
                    for neighbor in self.robot_neighbors[robot]:
                        topic = f"/{robot}/{neighbor}_estimated_position"
                        self.estimated_position_topics[robot][neighbor] = topic
            
            # Create list of robots that have neighbors for barycenter calculation
            self.robots_with_neighbors = []
            for robot, neighbors in self.robot_neighbors.items():
                if neighbors:  # Only if robot has neighbors
                    self.robots_with_neighbors.append(robot)
            
            print(f'Loaded {len(self.robot_names)} robots from config')
            print(f'Found {len(self.robots_with_neighbors)} robots with neighbors: {self.robots_with_neighbors}')
            
        except Exception as e:
            print(f'Error loading robot config: {e}')
            # Fallback to default configuration
            self.robot_names = ["Aramis", "Athos", "Porthos", "Dartagnan"]
            self.robot_neighbors = {
                "Aramis": ["Porthos", "Athos"],
                "Athos": ["Dartagnan", "Aramis"],
                "Dartagnan": ["Athos", "Porthos"],
                "Porthos": ["Aramis", "Dartagnan"]
            }
            self.robots_with_neighbors = ["Aramis", "Athos", "Dartagnan", "Porthos"]
            self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
            self.cmd_vel_topics = [f"/{name}/cmd_vel" for name in self.robot_names]
            
            # Create estimated position topics for fallback
            self.estimated_position_topics = {}
            for robot in self.robot_names:
                self.estimated_position_topics[robot] = {}
                if robot in self.robot_neighbors:
                    for neighbor in self.robot_neighbors[robot]:
                        topic = f"/{robot}/{neighbor}_estimated_position"
                        self.estimated_position_topics[robot][neighbor] = topic
