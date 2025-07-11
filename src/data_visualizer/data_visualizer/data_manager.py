import collections
import math
from transforms3d.euler import quat2euler

# Maximum history for the scrolling distance plot
MAX_TIME_HISTORY = 80  # seconds

class DataManager:
    def __init__(self, config):
        self.config = config
        self.update_interval = 0.1  # seconds
        
        # Store positions and trajectories
        self.trajectories = {name: {'x': [], 'y': [], 'time': []} for name in config.robot_names}
        self.current_positions = {name: None for name in config.robot_names}
        self.current_angles = {name: None for name in config.robot_names}
        self.current_velocities = {name: None for name in config.robot_names}
        self.last_displayed_positions = {name: None for name in config.robot_names}
        self.last_displayed_angles = {name: None for name in config.robot_names}
        
        # Store estimated positions of neighbors
        self.estimated_positions = {}
        for robot in config.robot_names:
            self.estimated_positions[robot] = {}
            if robot in config.robot_neighbors:
                for neighbor in config.robot_neighbors[robot]:
                    self.estimated_positions[robot][neighbor] = None
        
        # Distance history for plotting
        self.distance_history = {
            f"{name1}-{name2}": collections.deque(maxlen=int(MAX_TIME_HISTORY/self.update_interval))
            for i, name1 in enumerate(config.robot_names)
            for j, name2 in enumerate(config.robot_names) if i < j
        }
        self.time_history = collections.deque(maxlen=int(MAX_TIME_HISTORY/self.update_interval))
        
        # Store goal point
        self.goal_point = None
        
        # Store multi-level barycenters for each robot
        self.barycenters = {robot: {} for robot in config.robots_with_neighbors}
        
        # Store complete swarm barycenter
        self.swarm_barycenter = None
        
        # Start time for relative time tracking
        self.start_time = None
    
    def reset_distance_history(self):
        """Reset the distance history data"""
        for key in self.distance_history:
            self.distance_history[key].clear()
        self.time_history.clear()
        print("Distance history reset")
    
    def update_pose(self, name, msg):
        """Update robot pose data"""
        import time
        
        # Get current time
        current_time = time.time()
        
        # Initialize start time if first message
        if self.start_time is None:
            self.start_time = current_time
        
        # Calculate relative time
        relative_time = current_time - self.start_time
        
        # Extract position
        position = (msg.pose.position.x, msg.pose.position.y)
        self.current_positions[name] = position
        
        # Extract yaw angle from quaternion
        q = [msg.pose.orientation.w, msg.pose.orientation.x, 
             msg.pose.orientation.y, msg.pose.orientation.z]
        _, _, yaw = quat2euler(q)
        self.current_angles[name] = yaw
        
        # Add to trajectory
        self.trajectories[name]['x'].append(position[0])
        self.trajectories[name]['y'].append(position[1])
        self.trajectories[name]['time'].append(relative_time)
    
    def update_velocity(self, name, msg):
        """Update robot velocity data"""
        velocity = (msg.linear.x, msg.linear.y)
        self.current_velocities[name] = velocity
    
    def update_goal_point(self, msg):
        """Update goal point"""
        self.goal_point = (msg.x, msg.y)
    
    def update_estimated_position(self, robot, neighbor, msg):
        """Update estimated position of a neighbor as seen by a robot"""
        position = (msg.x, msg.y)
        self.estimated_positions[robot][neighbor] = position
    
    def calculate_barycenters_recursive(self, num_levels):
        """Calculate barycenters recursively for multiple levels"""
        # Clear previous calculations
        for robot in self.config.robots_with_neighbors:
            self.barycenters[robot].clear()
        
        # Level 1 (p1): Calculate barycenters for each robot with its neighbors
        for robot in self.config.robots_with_neighbors:
            neighbors = self.config.robot_neighbors[robot]
            
            # Check if robot and all its neighbors have positions
            if (self.current_positions[robot] and 
                all(self.current_positions[neighbor] for neighbor in neighbors)):
                
                # Get positions of robot and all its neighbors
                positions = [self.current_positions[robot]]
                positions.extend([self.current_positions[neighbor] for neighbor in neighbors])
                
                # Calculate barycenter (average of all positions)
                total_x = sum(pos[0] for pos in positions)
                total_y = sum(pos[1] for pos in positions)
                num_robots = len(positions)
                
                barycenter_x = total_x / num_robots
                barycenter_y = total_y / num_robots
                
                self.barycenters[robot][1] = (barycenter_x, barycenter_y)
        
        # Level 2 and beyond: Calculate meta-barycenters recursively
        for level in range(2, num_levels + 1):
            for robot in self.config.robots_with_neighbors:
                neighbors = self.config.robot_neighbors[robot]
                
                # Check if robot has its barycenter from previous level and all neighbors have theirs
                if (robot in self.barycenters and (level - 1) in self.barycenters[robot] and 
                    all(neighbor in self.barycenters and (level - 1) in self.barycenters[neighbor] 
                        for neighbor in neighbors)):
                    
                    # Get barycenter of robot and barycenters of all its neighbors from previous level
                    barycenter_positions = [self.barycenters[robot][level - 1]]
                    barycenter_positions.extend([self.barycenters[neighbor][level - 1] for neighbor in neighbors])
                    
                    # Calculate meta-barycenter (average of all barycenters from previous level)
                    total_x = sum(pos[0] for pos in barycenter_positions)
                    total_y = sum(pos[1] for pos in barycenter_positions)
                    num_barycenters = len(barycenter_positions)
                    
                    meta_barycenter_x = total_x / num_barycenters
                    meta_barycenter_y = total_y / num_barycenters
                    
                    self.barycenters[robot][level] = (meta_barycenter_x, meta_barycenter_y)
    
    def calculate_swarm_barycenter(self):
        """Calculate the barycenter of the complete swarm"""
        # Check if all robots have positions
        if all(self.current_positions[robot] for robot in self.config.robot_names):
            # Get all robot positions
            positions = [self.current_positions[robot] for robot in self.config.robot_names]
            
            # Calculate swarm barycenter (average of all robot positions)
            total_x = sum(pos[0] for pos in positions)
            total_y = sum(pos[1] for pos in positions)
            num_robots = len(positions)
            
            swarm_barycenter_x = total_x / num_robots
            swarm_barycenter_y = total_y / num_robots
            
            self.swarm_barycenter = (swarm_barycenter_x, swarm_barycenter_y)
        else:
            self.swarm_barycenter = None
    
    def update_distance_data(self):
        """Update distance history data"""
        if all(self.current_positions.values()):
            import time
            current_time = time.time()
            if self.start_time:
                relative_time = current_time - self.start_time
                
                self.time_history.append(relative_time)
                
                # Calculate and store distances
                for i, name1 in enumerate(self.config.robot_names):
                    for j, name2 in enumerate(self.config.robot_names):
                        if i < j:  # Avoid duplicates
                            x1, y1 = self.current_positions[name1]
                            x2, y2 = self.current_positions[name2]
                            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                            self.distance_history[f"{name1}-{name2}"].append(distance)
    
    def clear_trajectories(self):
        """Clear all trajectory data"""
        for name in self.config.robot_names:
            self.trajectories[name] = {'x': [], 'y': [], 'time': []}
    
    def positions_changed(self):
        """Check if positions have changed significantly"""
        positions_changed = False
        for name in self.config.robot_names:
            if (self.current_positions[name] and 
                (not self.last_displayed_positions[name] or
                 abs(self.current_positions[name][0] - self.last_displayed_positions[name][0]) > 0.001 or
                 abs(self.current_positions[name][1] - self.last_displayed_positions[name][1]) > 0.001 or
                 (self.current_angles[name] is not None and 
                  (self.last_displayed_angles[name] is None or
                   abs(self.current_angles[name] - self.last_displayed_angles[name]) > 0.01)))):
                positions_changed = True
                self.last_displayed_positions[name] = self.current_positions[name]
                self.last_displayed_angles[name] = self.current_angles[name]
        return positions_changed
