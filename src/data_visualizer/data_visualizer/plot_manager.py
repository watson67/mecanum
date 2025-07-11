import math
import matplotlib.pyplot as plt

MAX_TIME_HISTORY = 60  # seconds

class PlotManager:
    def __init__(self, config, data_manager, gui_manager):
        self.config = config
        self.data_manager = data_manager
        self.gui_manager = gui_manager
        
        # Plot elements will be initialized in setup_plots
        self.trajectory_lines = {}
        self.position_markers = {}
        self.orientation_arrows = {}
        self.velocity_arrows = {}
        self.barycenter_markers = {robot: {} for robot in config.robots_with_neighbors}
        self.robot_connection_lines = {}
        self.distance_lines = {}
        
        # Barycenter styling
        self.barycenter_colors = ['purple', 'orange', 'brown', 'pink', 'cyan', 'magenta', 'yellow', 'gray']
        self.barycenter_markers_style = ['s', 'D', 'v', '<', '>', 'h', 'p', '*','s', 'D']
        self.barycenter_sizes = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    
    def setup_plots(self, ax, distance_ax, canvas, distance_canvas):
        """Setup the plots with initial configuration"""
        self.ax = ax
        self.distance_ax = distance_ax
        self.canvas = canvas
        self.distance_canvas = distance_canvas
        
        self.setup_trajectory_plot()
        self.setup_distance_plot_elements()
    
    def setup_trajectory_plot(self):
        """Configure the initial trajectory plot"""
        self.ax.set_ylabel('X Position (m)')
        self.ax.set_xlabel('Y Position (m)')
        self.ax.set_title('Robot Trajectories and Velocities')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(2, -2)
        self.ax.set_ylim(-2, 2)
        
        # Create plot elements for each robot
        for i, name in enumerate(self.config.robot_names):
            color = self.config.colors[i % len(self.config.colors)]
            
            # Trajectory line
            line, = self.ax.plot([], [], '-', color=color, label=f'{name}', linewidth=2, alpha=0.7)
            self.trajectory_lines[name] = line
            
            # Position marker
            marker, = self.ax.plot([], [], 'o', color=color, markersize=8)
            self.position_markers[name] = marker
            
            # Orientation arrow
            arrow = self.ax.quiver(0, 0, 0, 0, color=color, alpha=0.5, width=0.003)
            self.orientation_arrows[name] = arrow
            
            # Velocity arrow
            vel_arrow = self.ax.quiver(0, 0, 0, 0, color=color, alpha=1.0, width=0.006)
            self.velocity_arrows[name] = vel_arrow
        
        # Goal point marker
        self.goal_marker, = self.ax.plot([], [], 'x', color='black', markersize=15, markeredgewidth=3, label='Goal')
        
        # Create barycenter markers
        self.setup_barycenter_markers()
        
        # Create swarm barycenter marker
        self.swarm_barycenter_marker, = self.ax.plot([], [], '^', color='red', markersize=10, alpha=0.9, 
                                                    label='Swarm Barycenter')
        
        # Create connection lines
        self.setup_connection_lines()
        
        # Add legend
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Use the figure from the canvas for tight_layout
        self.canvas.figure.tight_layout()
    
    def setup_barycenter_markers(self):
        """Setup barycenter markers for all levels"""
        for i, robot in enumerate(self.config.robots_with_neighbors):
            for level in range(1, self.gui_manager.max_barycenter_levels + 1):
                color = self.barycenter_colors[(level - 1) % len(self.barycenter_colors)]
                marker_style = self.barycenter_markers_style[(level - 1) % len(self.barycenter_markers_style)]
                size = self.barycenter_sizes[(level - 1) % len(self.barycenter_sizes)]
                
                marker, = self.ax.plot([], [], marker_style, color=color, markersize=size, alpha=0.7,
                                     label=f'P{level} Barycenter' if i == 0 else "", visible=False)
                self.barycenter_markers[robot][level] = marker
    
    def setup_connection_lines(self):
        """Setup connection lines between robots"""
        for robot in self.config.robots_with_neighbors:
            neighbors = self.config.robot_neighbors[robot]
            for neighbor in neighbors:
                connection_key = f"{robot}-{neighbor}"
                reverse_key = f"{neighbor}-{robot}"
                if connection_key not in self.robot_connection_lines and reverse_key not in self.robot_connection_lines:
                    line, = self.ax.plot([], [], 'k--', linewidth=1, alpha=0.5, visible=False)
                    self.robot_connection_lines[connection_key] = line
    
    def setup_distance_plot_elements(self):
        """Setup distance plot elements"""
        self.distance_ax.set_xlabel('Time (s)')
        self.distance_ax.set_ylabel('Distance (m)')
        self.distance_ax.set_title('Robot Distances Over Time')
        self.distance_ax.grid(True)
        
        # Create lines for each robot pair
        for i, name1 in enumerate(self.config.robot_names):
            for j, name2 in enumerate(self.config.robot_names):
                if i < j:
                    pair_name = f"{name1}-{name2}"
                    line, = self.distance_ax.plot([], [], '-', label=pair_name, linewidth=2)
                    self.distance_lines[pair_name] = line
        
        self.distance_ax.legend(loc='upper right')
    
    def update_trajectory_plot(self):
        """Update the trajectory plot with current data"""
        plot_needs_update = False
        
        # Update trajectories
        for name in self.config.robot_names:
            x_data = self.data_manager.trajectories[name]['x']
            y_data = self.data_manager.trajectories[name]['y']
            
            if x_data and y_data and self.gui_manager.draw_trajectories:
                self.trajectory_lines[name].set_data(y_data, x_data)
                self.trajectory_lines[name].set_visible(True)
                plot_needs_update = True
            elif not self.gui_manager.draw_trajectories:
                self.trajectory_lines[name].set_visible(False)
                plot_needs_update = True
        
        # Update positions and arrows
        plot_needs_update |= self.update_robot_markers()
        
        # Update barycenters
        plot_needs_update |= self.update_barycenters()
        
        # Update goal point
        plot_needs_update |= self.update_goal_marker()
        
        # Update connections
        plot_needs_update |= self.update_connections()
        
        if plot_needs_update:
            self.ax.set_xlim(2, -2)
            self.ax.set_ylim(-2, 2)
            self.canvas.draw_idle()
        
        return plot_needs_update
    
    def update_robot_markers(self):
        """Update robot position markers and arrows"""
        plot_needs_update = False
        
        for name in self.config.robot_names:
            if self.data_manager.current_positions[name]:
                x_pos, y_pos = self.data_manager.current_positions[name]
                self.position_markers[name].set_data([y_pos], [x_pos])
                
                # Update orientation arrow
                if self.data_manager.current_angles[name] is not None:
                    arrow_length = 0.2
                    dx = arrow_length * math.sin(self.data_manager.current_angles[name])
                    dy = arrow_length * math.cos(self.data_manager.current_angles[name])
                    
                    self.orientation_arrows[name].remove()
                    self.orientation_arrows[name] = self.ax.quiver(
                        y_pos, x_pos, dx, dy, 
                        color=self.trajectory_lines[name].get_color(),
                        alpha=0.5, width=0.003, scale=1, scale_units='xy', angles='xy'
                    )
                
                # Update velocity arrow
                if self.data_manager.current_velocities[name] is not None:
                    vel_x, vel_y = self.data_manager.current_velocities[name]
                    velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
                    
                    if velocity_magnitude > 0.01:
                        self.velocity_arrows[name].remove()
                        self.velocity_arrows[name] = self.ax.quiver(
                            y_pos, x_pos, vel_y, vel_x,
                            color=self.trajectory_lines[name].get_color(),
                            alpha=1.0, width=0.006, scale=1, scale_units='xy', angles='xy'
                        )
                    else:
                        self.velocity_arrows[name].remove()
                        self.velocity_arrows[name] = self.ax.quiver(
                            0, 0, 0, 0, 
                            color=self.trajectory_lines[name].get_color(),
                            alpha=1.0, width=0.006
                        )
                
                plot_needs_update = True
        
        return plot_needs_update
    
    def update_barycenters(self):
        """Update barycenter markers"""
        plot_needs_update = False
        
        # Calculate barycenters
        self.data_manager.calculate_barycenters_recursive(self.gui_manager.num_barycenter_levels)
        self.data_manager.calculate_swarm_barycenter()
        
        if self.gui_manager.draw_barycenters:
            # Update individual robot barycenters
            for robot in self.config.robots_with_neighbors:
                for level in range(1, self.gui_manager.num_barycenter_levels + 1):
                    if level in self.data_manager.barycenters[robot]:
                        bx, by = self.data_manager.barycenters[robot][level]
                        self.barycenter_markers[robot][level].set_data([by], [bx])
                        self.barycenter_markers[robot][level].set_visible(True)
                        plot_needs_update = True
                    else:
                        self.barycenter_markers[robot][level].set_visible(False)
                
                # Hide markers beyond current level
                for level in range(self.gui_manager.num_barycenter_levels + 1, self.gui_manager.max_barycenter_levels + 1):
                    self.barycenter_markers[robot][level].set_visible(False)
            
            # Update swarm barycenter
            if self.data_manager.swarm_barycenter:
                sbx, sby = self.data_manager.swarm_barycenter
                self.swarm_barycenter_marker.set_data([sby], [sbx])
                self.swarm_barycenter_marker.set_visible(True)
                plot_needs_update = True
        else:
            # Hide all barycenter markers
            for robot in self.config.robots_with_neighbors:
                for level in range(1, self.gui_manager.max_barycenter_levels + 1):
                    self.barycenter_markers[robot][level].set_visible(False)
            self.swarm_barycenter_marker.set_visible(False)
            plot_needs_update = True
        
        return plot_needs_update
    
    def update_goal_marker(self):
        """Update goal point marker"""
        if self.data_manager.goal_point:
            goal_x, goal_y = self.data_manager.goal_point
            self.goal_marker.set_data([goal_y], [goal_x])
            return True
        return False
    
    def update_connections(self):
        """Update connection lines between robots"""
        plot_needs_update = False
        
        if self.gui_manager.draw_robot_connections:
            for robot in self.config.robots_with_neighbors:
                neighbors = self.config.robot_neighbors[robot]
                for neighbor in neighbors:
                    connection_key = f"{robot}-{neighbor}"
                    reverse_key = f"{neighbor}-{robot}"
                    
                    if (self.data_manager.current_positions[robot] and 
                        self.data_manager.current_positions[neighbor]):
                        x1, y1 = self.data_manager.current_positions[robot]
                        x2, y2 = self.data_manager.current_positions[neighbor]
                        
                        if connection_key in self.robot_connection_lines:
                            line = self.robot_connection_lines[connection_key]
                            line.set_data([y1, y2], [x1, x2])
                            line.set_visible(True)
                            plot_needs_update = True
                        elif reverse_key in self.robot_connection_lines:
                            line = self.robot_connection_lines[reverse_key]
                            line.set_data([y2, y1], [x2, x1])
                            line.set_visible(True)
                            plot_needs_update = True
        
        return plot_needs_update
    
    def update_distance_plot(self):
        """Update the distance history plot"""
        if len(self.data_manager.time_history) < 2:
            return
        
        # Update each distance line
        for pair_name, line in self.distance_lines.items():
            line.set_data(list(self.data_manager.time_history), 
                         list(self.data_manager.distance_history[pair_name]))
        
        # Adjust axes
        if self.data_manager.time_history:
            x_min = max(0, self.data_manager.time_history[-1] - MAX_TIME_HISTORY)
            x_max = self.data_manager.time_history[-1] + 1
            self.distance_ax.set_xlim(x_min, x_max)
            
            # Find min and max distance for y-axis
            all_distances = []
            for distances in self.data_manager.distance_history.values():
                all_distances.extend(list(distances))
            
            if all_distances:
                min_dist = max(0, min(all_distances) - 0.1)
                max_dist = max(all_distances) + 0.1
                self.distance_ax.set_ylim(min_dist, max_dist)
        
        self.distance_canvas.draw_idle()
    
    def clear_trajectory_display(self):
        """Clear trajectory display"""
        for name in self.config.robot_names:
            self.trajectory_lines[name].set_data([], [])
        self.canvas.draw_idle()
    
    def update_connection_visibility(self):
        """Update visibility of connection lines"""
        if not self.gui_manager.draw_robot_connections:
            for line in self.robot_connection_lines.values():
                line.set_visible(False)
            self.canvas.draw_idle()
    
    def update_barycenter_visibility(self):
        """Update visibility of barycenter markers"""
        for robot in self.config.robots_with_neighbors:
            for level in range(1, self.gui_manager.max_barycenter_levels + 1):
                if level <= self.gui_manager.num_barycenter_levels and self.gui_manager.draw_barycenters:
                    self.barycenter_markers[robot][level].set_visible(True)
                else:
                    self.barycenter_markers[robot][level].set_visible(False)
        
        self.swarm_barycenter_marker.set_visible(self.gui_manager.draw_barycenters)
        self.canvas.draw_idle()
    
    def update_trajectory_visibility(self):
        """Update visibility of trajectory lines"""
        for name in self.config.robot_names:
            self.trajectory_lines[name].set_visible(self.gui_manager.draw_trajectories)
        self.canvas.draw_idle()
