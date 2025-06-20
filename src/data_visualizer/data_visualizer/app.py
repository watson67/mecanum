#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
import math
import threading
import collections
from transforms3d.euler import quat2euler
import yaml
import os

# Maximum history for the scrolling distance plot
MAX_TIME_HISTORY = 60  # seconds

class RobotVisualizerApp(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # Load robot configuration from YAML
        self.load_robot_config()
        
        # Configuration
        self.colors = ['red', 'green', 'blue', 'orange']
        self.pose_topics = [f"/vrpn_mocap/{name}/pose" for name in self.robot_names]
        self.cmd_vel_topics = [f"/{name}/cmd_vel" for name in self.robot_names]
        
        # QoS profile for MOCAP
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Store positions and trajectories
        self.trajectories = {name: {'x': [], 'y': [], 'time': []} for name in self.robot_names}
        self.current_positions = {name: None for name in self.robot_names}
        self.current_angles = {name: None for name in self.robot_names}
        self.current_velocities = {name: None for name in self.robot_names}
        self.last_displayed_positions = {name: None for name in self.robot_names}
        self.last_displayed_angles = {name: None for name in self.robot_names}
        
        # Define update interval before using it
        self.update_interval = 0.1  # seconds
        
        # Distance history for plotting
        self.distance_history = {
            f"{name1}-{name2}": collections.deque(maxlen=int(MAX_TIME_HISTORY/self.update_interval))
            for i, name1 in enumerate(self.robot_names)
            for j, name2 in enumerate(self.robot_names) if i < j
        }
        self.time_history = collections.deque(maxlen=int(MAX_TIME_HISTORY/self.update_interval))
        
        # Flag for drawing lines between robots
        self.draw_robot_connections = False
        self.robot_connection_lines = {}
        
        # Flag for drawing trajectories
        self.draw_trajectories = True
        
        # Flag for drawing barycenters
        self.draw_barycenters = True
        
        # Number of barycenter levels to calculate (p1, p2, p3, ...)
        self.num_barycenter_levels = 3
        self.max_barycenter_levels = 10
        
        # Store goal point
        self.goal_point = None
        
        # Store multi-level barycenters for each robot
        # Structure: self.barycenters[robot][level] = (x, y)
        self.barycenters = {robot: {} for robot in self.robots_with_neighbors}
        self.barycenter_markers = {robot: {} for robot in self.robots_with_neighbors}
        
        # Colors and markers for different barycenter levels
        self.barycenter_colors = ['purple', 'orange', 'brown', 'pink', 'cyan', 'magenta', 'yellow', 'gray']
        self.barycenter_markers_style = ['s', 'D', 'v', '<', '>', 'h', 'p', '*','s', 'D']
        self.barycenter_sizes = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        
        # Store complete swarm barycenter
        self.swarm_barycenter = None
        self.swarm_barycenter_marker = None
        
        # Subscribe to pose topics
        for i, topic in enumerate(self.pose_topics):
            self.create_subscription(
                PoseStamped,
                topic,
                self.pose_callback_factory(self.robot_names[i]),
                qos_profile=self.qos_profile
            )
        
        # Subscribe to cmd_vel topics
        for i, topic in enumerate(self.cmd_vel_topics):
            self.create_subscription(
                Twist,
                topic,
                self.cmd_vel_callback_factory(self.robot_names[i]),
                10
            )
        
        # Subscribe to goal_point topic
        self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10
        )
        
        # Create and setup the GUI
        self.setup_gui()
        
        # Timer for updating the plot
        self.timer = self.create_timer(self.update_interval, self.update_plot)
        
        # Start time for relative time tracking
        self.start_time = None
        
        # Flag to track if GUI needs text update
        self.text_update_needed = True
        
        self.get_logger().info('Robot visualizer started')
    
    def load_robot_config(self):
        """Load robot configuration from YAML file"""
        try:
            # Get the path to the YAML file
            config_path = "/home/eswarm/mecanum/src/mecanum_swarm/config/robots.yaml"
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            self.robot_names = config['all_robot_names']
            self.robot_neighbors = config['robot_neighbors']
            
            # Create list of robots that have neighbors for barycenter calculation
            self.robots_with_neighbors = []
            for robot, neighbors in self.robot_neighbors.items():
                if neighbors:  # Only if robot has neighbors
                    self.robots_with_neighbors.append(robot)
            
            self.get_logger().info(f'Loaded {len(self.robot_names)} robots from config')
            self.get_logger().info(f'Found {len(self.robots_with_neighbors)} robots with neighbors: {self.robots_with_neighbors}')
            
        except Exception as e:
            self.get_logger().error(f'Error loading robot config: {e}')
            # Fallback to default configuration
            self.robot_names = ["Aramis", "Athos", "Porthos", "Dartagnan"]
            self.robot_neighbors = {
                "Aramis": ["Porthos", "Athos"],
                "Athos": ["Dartagnan", "Aramis"],
                "Dartagnan": ["Athos", "Porthos"],
                "Porthos": ["Aramis", "Dartagnan"]
            }
            self.robots_with_neighbors = ["Aramis", "Athos", "Dartagnan", "Porthos"]
    
    def setup_gui(self):
        # Create main window
        self.root = tk.Tk()
        self.root.title("Robot Position Visualizer")
        self.root.geometry("1200x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Configure the main window to be resizable
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        
        # Create main layout frame with grid
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.main_frame.columnconfigure(0, weight=3)  # Plot gets 3/4
        self.main_frame.columnconfigure(1, weight=1)  # Data gets 1/4
        self.main_frame.rowconfigure(0, weight=1)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.grid(row=0, column=0, sticky="nsew")
        
        # Tab 1: Trajectory Plot
        self.trajectory_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.trajectory_tab, text="Trajectories")
        self.trajectory_tab.rowconfigure(0, weight=1)
        self.trajectory_tab.columnconfigure(0, weight=1)
        
        # Tab 2: Distance History
        self.distance_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.distance_tab, text="Distance History")
        self.distance_tab.rowconfigure(0, weight=1)
        self.distance_tab.columnconfigure(0, weight=1)
        
        # Setup the trajectory plot
        self.setup_trajectory_plot()
        
        # Setup the distance history plot
        self.setup_distance_plot()
        
        # Right frame for the data display (using grid for stability)
        self.data_frame = ttk.Frame(self.main_frame)
        self.data_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        self.data_frame.rowconfigure(0, weight=1)
        self.data_frame.columnconfigure(0, weight=1)
        
        # Setup the data display
        self.setup_data_display()
    
    def setup_trajectory_plot(self):
        """Setup the trajectory plot tab"""
        # Left frame for the plot
        self.plot_frame = ttk.Frame(self.trajectory_tab)
        self.plot_frame.grid(row=0, column=0, sticky="nsew")
        self.plot_frame.rowconfigure(0, weight=1)
        self.plot_frame.rowconfigure(1, weight=0)
        self.plot_frame.columnconfigure(0, weight=1)

        # Create a frame for the matplotlib plot
        self.matplotlib_frame = ttk.Frame(self.plot_frame)
        self.matplotlib_frame.grid(row=0, column=0, sticky="nsew")

        # Setup the matplotlib figure and canvas
        self.fig = Figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.matplotlib_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Add navigation toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.matplotlib_frame)
        self.toolbar.update()

        # Setup the plot
        self.setup_plot()

        # Create a separate frame for buttons
        self.button_frame = ttk.LabelFrame(self.plot_frame, text="Controls")
        self.button_frame.grid(row=1, column=0, sticky="ew", pady=10, padx=10)

        # Clear trajectory button
        self.clear_button = ttk.Button(
            self.button_frame, 
            text="Clear Trajectories",
            command=self.clear_trajectories
        )
        self.clear_button.grid(row=0, column=0, padx=5, pady=5)
        
        # Toggle robot connections button
        self.connect_button = ttk.Button(
            self.button_frame, 
            text="Toggle Robot Connections",
            command=self.toggle_robot_connections
        )
        self.connect_button.grid(row=0, column=1, padx=5, pady=5)
        
        # Toggle trajectories button
        self.trajectory_button = ttk.Button(
            self.button_frame, 
            text="Toggle Trajectories",
            command=self.toggle_trajectories
        )
        self.trajectory_button.grid(row=0, column=2, padx=5, pady=5)
        
        # Toggle barycenters button
        self.barycenter_button = ttk.Button(
            self.button_frame, 
            text="Toggle Barycenters",
            command=self.toggle_barycenters
        )
        self.barycenter_button.grid(row=0, column=3, padx=5, pady=5)
        
        # Barycenter level controls
        self.level_frame = ttk.Frame(self.button_frame)
        self.level_frame.grid(row=1, column=0, columnspan=4, pady=5)
        
        ttk.Label(self.level_frame, text="Barycenter Levels:").pack(side=tk.LEFT, padx=5)
        
        self.level_var = tk.IntVar(value=self.num_barycenter_levels)
        self.level_spinbox = tk.Spinbox(
            self.level_frame, 
            from_=1, 
            to=self.max_barycenter_levels, 
            width=5,
            textvariable=self.level_var,
            command=self.update_barycenter_levels
        )
        self.level_spinbox.pack(side=tk.LEFT, padx=5)
        
        # Bind spinbox change event
        self.level_var.trace('w', self.on_level_change)
    
    def setup_distance_plot(self):
        """Setup the distance history plot tab"""
        self.distance_fig = Figure(figsize=(8, 6))
        self.distance_ax = self.distance_fig.add_subplot(111)
        self.distance_canvas = FigureCanvasTkAgg(self.distance_fig, master=self.distance_tab)
        self.distance_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add navigation toolbar
        self.distance_toolbar = NavigationToolbar2Tk(self.distance_canvas, self.distance_tab)
        self.distance_toolbar.update()
        
        # Setup the distance plot
        self.distance_ax.set_xlabel('Time (s)')
        self.distance_ax.set_ylabel('Distance (m)')
        self.distance_ax.set_title('Robot Distances Over Time')
        self.distance_ax.grid(True)
        
        # Create lines for each robot pair
        self.distance_lines = {}
        robot_pairs = []
        for i, name1 in enumerate(self.robot_names):
            for j, name2 in enumerate(self.robot_names):
                if i < j:
                    pair_name = f"{name1}-{name2}"
                    robot_pairs.append(pair_name)
                    line, = self.distance_ax.plot([], [], '-', label=pair_name, linewidth=2)
                    self.distance_lines[pair_name] = line
        
        # Add legend
        self.distance_ax.legend(loc='upper right')
        
    def setup_plot(self):
        """Configure the initial plot"""
        self.ax.set_ylabel('X Position (m)')  # X on Y-axis
        self.ax.set_xlabel('Y Position (m)')  # Y on X-axis
        self.ax.set_title('Robot Trajectories and Velocities')
        
        # Set grid
        self.ax.grid(True)
        
        # Equal scaling
        self.ax.set_aspect('equal')
        
        # Set fixed limits
        self.ax.set_xlim(2, -2)  # Inverted to have y to the left
        self.ax.set_ylim(-2, 2)
        
        # Create lines and markers for each robot
        self.trajectory_lines = {}
        self.position_markers = {}
        self.orientation_arrows = {}
        self.velocity_arrows = {}  # Add velocity arrows
        
        for i, name in enumerate(self.robot_names):
            color = self.colors[i % len(self.colors)]
            
            # Line for the trajectory
            line, = self.ax.plot([], [], '-', color=color, label=f'{name}', linewidth=2, alpha=0.7)
            self.trajectory_lines[name] = line
            
            # Marker for the current position (plus grand)
            marker, = self.ax.plot([], [], 'o', color=color, markersize=8)
            self.position_markers[name] = marker
            
            # Arrow for orientation (plus grand)
            arrow = self.ax.quiver(0, 0, 0, 0, color=color, alpha=0.5, width=0.003)
            self.orientation_arrows[name] = arrow
            
            # Arrow for velocity (plus grand)
            vel_arrow = self.ax.quiver(0, 0, 0, 0, color=color, alpha=1.0, width=0.006)
            self.velocity_arrows[name] = vel_arrow
        
        # Goal point marker
        self.goal_marker, = self.ax.plot([], [], 'x', color='black', markersize=15, markeredgewidth=3, label='Goal')
        
        # Create barycenter markers for each robot with multiple levels
        for i, robot in enumerate(self.robots_with_neighbors):
            for level in range(1, self.max_barycenter_levels + 1):
                color = self.barycenter_colors[(level - 1) % len(self.barycenter_colors)]
                marker_style = self.barycenter_markers_style[(level - 1) % len(self.barycenter_markers_style)]
                size = self.barycenter_sizes[(level - 1) % len(self.barycenter_sizes)]
                
                marker, = self.ax.plot([], [], marker_style, color=color, markersize=size, alpha=0.7,
                                     label=f'P{level} Barycenter' if i == 0 else "", visible=False)
                self.barycenter_markers[robot][level] = marker
        
        # Create swarm barycenter marker
        self.swarm_barycenter_marker, = self.ax.plot([], [], '^', color='red', markersize=10, alpha=0.9, 
                                                    label='Swarm Barycenter')
        
        # Create connection lines between robots (initially invisible)
        for robot in self.robots_with_neighbors:
            neighbors = self.robot_neighbors[robot]
            for neighbor in neighbors:
                # Only create connection if it doesn't already exist (avoid duplicates)
                connection_key = f"{robot}-{neighbor}"
                reverse_key = f"{neighbor}-{robot}"
                if connection_key not in self.robot_connection_lines and reverse_key not in self.robot_connection_lines:
                    line, = self.ax.plot([], [], 'k--', linewidth=1, alpha=0.5, visible=False)
                    self.robot_connection_lines[connection_key] = line
        
        # Add legend outside the plot area
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Adjust layout to accommodate the legend
        self.fig.tight_layout()
    
    def setup_data_display(self):
        """Create the data display area"""
        # Create a LabelFrame for better visual separation
        self.data_label_frame = ttk.LabelFrame(self.data_frame, text="Robot Data")
        self.data_label_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create a text widget for displaying data
        self.data_text = tk.Text(self.data_label_frame, wrap=tk.WORD, height=30, width=40, font=("Courier", 10))
        self.data_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(self.data_label_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Configure text widget and scrollbar
        self.data_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.data_text.yview)
        
        # Make the text widget read-only
        self.data_text.config(state=tk.DISABLED)
    
    def pose_callback_factory(self, name):
        def callback(msg):
            # Get current time
            current_time = self.get_clock().now().nanoseconds / 1e9
            
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
            
            # Flag that we need a text update
            self.text_update_needed = True
            
            self.get_logger().debug(f'{name} position: ({position[0]:.2f}, {position[1]:.2f}), yaw: {yaw:.2f}')
        
        return callback
    
    def cmd_vel_callback_factory(self, name):
        def callback(msg):
            # Store velocity data
            velocity = (msg.linear.x, msg.linear.y)
            self.current_velocities[name] = velocity
            
            self.get_logger().debug(f'{name} velocity: ({velocity[0]:.2f}, {velocity[1]:.2f})')
        
        return callback
    
    def goal_point_callback(self, msg):
        """Callback for goal point messages"""
        self.goal_point = (msg.x, msg.y)
        self.get_logger().info(f'Goal point updated: ({msg.x:.2f}, {msg.y:.2f})')
    
    def update_barycenter_levels(self):
        """Update the number of barycenter levels to calculate"""
        self.num_barycenter_levels = self.level_var.get()
        self.get_logger().info(f'Barycenter levels set to: {self.num_barycenter_levels}')
        
        # Update marker visibility
        self.update_barycenter_marker_visibility()
    
    def on_level_change(self, *args):
        """Handle level variable change"""
        self.update_barycenter_levels()
    
    def update_barycenter_marker_visibility(self):
        """Update visibility of barycenter markers based on current level setting"""
        for robot in self.robots_with_neighbors:
            for level in range(1, self.max_barycenter_levels + 1):
                if level <= self.num_barycenter_levels and self.draw_barycenters:
                    self.barycenter_markers[robot][level].set_visible(True)
                else:
                    self.barycenter_markers[robot][level].set_visible(False)
        
        self.canvas.draw_idle()
    
    def calculate_barycenters_recursive(self):
        """Calculate barycenters recursively for multiple levels"""
        # Clear previous calculations
        for robot in self.robots_with_neighbors:
            self.barycenters[robot].clear()
        
        # Level 1 (p1): Calculate barycenters for each robot with its neighbors
        for robot in self.robots_with_neighbors:
            neighbors = self.robot_neighbors[robot]
            
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
        for level in range(2, self.num_barycenter_levels + 1):
            for robot in self.robots_with_neighbors:
                neighbors = self.robot_neighbors[robot]
                
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
        if all(self.current_positions[robot] for robot in self.robot_names):
            # Get all robot positions
            positions = [self.current_positions[robot] for robot in self.robot_names]
            
            # Calculate swarm barycenter (average of all robot positions)
            total_x = sum(pos[0] for pos in positions)
            total_y = sum(pos[1] for pos in positions)
            num_robots = len(positions)
            
            swarm_barycenter_x = total_x / num_robots
            swarm_barycenter_y = total_y / num_robots
            
            self.swarm_barycenter = (swarm_barycenter_x, swarm_barycenter_y)
        else:
            self.swarm_barycenter = None
    
    def update_plot(self):
        """Update the plot with current positions and trajectories"""
        try:
            plot_needs_update = False
            
            # Update real trajectories
            for name in self.robot_names:
                # Get trajectory data
                x_data = self.trajectories[name]['x']
                y_data = self.trajectories[name]['y']
                
                # Update trajectory line (invert x and y for plotting)
                if x_data and y_data and self.draw_trajectories:
                    self.trajectory_lines[name].set_data(y_data, x_data)
                    self.trajectory_lines[name].set_visible(True)
                    plot_needs_update = True
                elif not self.draw_trajectories:
                    self.trajectory_lines[name].set_visible(False)
                    plot_needs_update = True
                    
                # Update position marker
                if self.current_positions[name]:
                    x_pos, y_pos = self.current_positions[name]
                    self.position_markers[name].set_data([y_pos], [x_pos])
                    
                    # Update orientation arrow if we have angle data
                    if self.current_angles[name] is not None:
                        # Calculate arrow vector components (plus grand)
                        arrow_length = 0.2
                        dx = arrow_length * math.sin(self.current_angles[name])
                        dy = arrow_length * math.cos(self.current_angles[name])
                        
                        # Remove old arrow and create new one at the current position
                        self.orientation_arrows[name].remove()
                        self.orientation_arrows[name] = self.ax.quiver(
                            y_pos, x_pos, dx, dy, 
                            color=self.trajectory_lines[name].get_color(),
                            alpha=0.5, width=0.003, scale=1, scale_units='xy', angles='xy'
                        )
                    
                    # Update velocity arrow if we have velocity data
                    if self.current_velocities[name] is not None:
                        vel_x, vel_y = self.current_velocities[name]
                        # Only show velocity vector if robot is moving
                        velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
                        
                        if velocity_magnitude > 0.01:  # Threshold to avoid tiny arrows
                            # Remove old velocity arrow and create new one (plus grand)
                            self.velocity_arrows[name].remove()
                            self.velocity_arrows[name] = self.ax.quiver(
                                y_pos, x_pos, vel_y, vel_x,  # Note: swapped for plot coordinates
                                color=self.trajectory_lines[name].get_color(),
                                alpha=1.0, width=0.006, scale=1, scale_units='xy', angles='xy'
                            )
                        else:
                            # Hide arrow if robot is not moving
                            self.velocity_arrows[name].remove()
                            self.velocity_arrows[name] = self.ax.quiver(
                                0, 0, 0, 0, 
                                color=self.trajectory_lines[name].get_color(),
                                alpha=1.0, width=0.006
                            )
            
            # Calculate and update multi-level barycenters
            self.calculate_barycenters_recursive()
            if self.draw_barycenters:
                for robot in self.robots_with_neighbors:
                    for level in range(1, self.num_barycenter_levels + 1):
                        if level in self.barycenters[robot]:
                            bx, by = self.barycenters[robot][level]
                            self.barycenter_markers[robot][level].set_data([by], [bx])  # Note: swapped for plot coordinates
                            self.barycenter_markers[robot][level].set_visible(True)
                            plot_needs_update = True
                        else:
                            self.barycenter_markers[robot][level].set_visible(False)
                
                # Hide markers for levels beyond current setting
                for robot in self.robots_with_neighbors:
                    for level in range(self.num_barycenter_levels + 1, self.max_barycenter_levels + 1):
                        self.barycenter_markers[robot][level].set_visible(False)
            else:
                # Hide all barycenter markers
                for robot in self.robots_with_neighbors:
                    for level in range(1, self.max_barycenter_levels + 1):
                        self.barycenter_markers[robot][level].set_visible(False)
                plot_needs_update = True
            
            # Calculate and update swarm barycenter
            self.calculate_swarm_barycenter()
            if self.swarm_barycenter and self.draw_barycenters:
                sbx, sby = self.swarm_barycenter
                self.swarm_barycenter_marker.set_data([sby], [sbx])  # Note: swapped for plot coordinates
                self.swarm_barycenter_marker.set_visible(True)
                plot_needs_update = True
            else:
                self.swarm_barycenter_marker.set_visible(False)
                plot_needs_update = True
            
            # Update goal point marker
            if self.goal_point:
                goal_x, goal_y = self.goal_point
                self.goal_marker.set_data([goal_y], [goal_x])  # Note: swapped for plot coordinates
                plot_needs_update = True
            
            # Update connection lines if enabled
            if self.draw_robot_connections:
                for robot in self.robots_with_neighbors:
                    neighbors = self.robot_neighbors[robot]
                    for neighbor in neighbors:
                        # Check both possible connection keys
                        connection_key = f"{robot}-{neighbor}"
                        reverse_key = f"{neighbor}-{robot}"
                        
                        if (self.current_positions[robot] and self.current_positions[neighbor]):
                            x1, y1 = self.current_positions[robot]
                            x2, y2 = self.current_positions[neighbor]
                            
                            # Use whichever connection exists
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
            
            # Only redraw if needed
            if plot_needs_update:
                # Keep fixed limits
                self.ax.set_xlim(2, -2)
                self.ax.set_ylim(-2, 2)
                
                # Update the canvas
                self.canvas.draw_idle()
            
            # Update position information only if needed
            if self.text_update_needed:
                self.update_position_info()
                self.text_update_needed = False
            
            # Update distance plot
            if all(self.current_positions.values()):
                # Add current time point
                current_time = self.get_clock().now().nanoseconds / 1e9
                if self.start_time:
                    relative_time = current_time - self.start_time
                    
                    self.time_history.append(relative_time)
                    
                    # Calculate and store distances
                    for i, name1 in enumerate(self.robot_names):
                        for j, name2 in enumerate(self.robot_names):
                            if i < j:  # Avoid duplicates
                                x1, y1 = self.current_positions[name1]
                                x2, y2 = self.current_positions[name2]
                                distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                                self.distance_history[f"{name1}-{name2}"].append(distance)
                    
                    # Update distance plot if we're on that tab
                    if self.notebook.index(self.notebook.select()) == 1:  # Distance tab is selected
                        self.update_distance_plot()
        
        except Exception as e:
            self.get_logger().error(f'Error updating plot: {e}')
    
    def update_distance_plot(self):
        """Update the distance history plot"""
        # Check if we have enough data
        if len(self.time_history) < 2:
            return
        
        # Update each distance line
        for pair_name, line in self.distance_lines.items():
            line.set_data(list(self.time_history), list(self.distance_history[pair_name]))
        
        # Adjust x-axis limits to show scrolling effect
        if self.time_history:
            x_min = max(0, self.time_history[-1] - MAX_TIME_HISTORY)
            x_max = self.time_history[-1] + 1
            self.distance_ax.set_xlim(x_min, x_max)
            
            # Find min and max distance for y-axis with some padding
            all_distances = []
            for distances in self.distance_history.values():
                all_distances.extend(list(distances))
            
            if all_distances:
                min_dist = max(0, min(all_distances) - 0.1)
                max_dist = max(all_distances) + 0.1
                self.distance_ax.set_ylim(min_dist, max_dist)
        
        # Update the canvas
        self.distance_canvas.draw_idle()
    
    def update_position_info(self):
        """Update the position and velocity information in the text area"""
        if not all(self.current_positions.values()):
            return  # Wait until all positions are available
        
        # Check if positions have changed significantly
        positions_changed = False
        for name in self.robot_names:
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
        
        if not positions_changed:
            return  # No need to update if positions haven't changed
            
        # Build text with position and velocity information
        info_text = "Robot Data:\n\n"
        
        # Format for positions including yaw and velocity
        robot_format = "{name:8}:\n  Position: X={x:8.3f} m, Y={y:8.3f} m\n  Yaw:      {yaw:8.2f} deg\n  Velocity: Vx={vx:7.3f} m/s, Vy={vy:7.3f} m/s\n\n"
        
        for name in self.robot_names:
            if self.current_positions[name] and self.current_angles[name] is not None:
                x, y = self.current_positions[name]
                yaw_deg = math.degrees(self.current_angles[name])
                
                # Get velocity data
                vx, vy = (0.0, 0.0)
                if self.current_velocities[name] is not None:
                    vx, vy = self.current_velocities[name]
                
                info_text += robot_format.format(
                    name=name, x=x, y=y, yaw=yaw_deg, vx=vx, vy=vy
                )
        
        # Add goal point information
        if self.goal_point:
            goal_x, goal_y = self.goal_point
            info_text += f"Goal Point: X={goal_x:8.3f} m, Y={goal_y:8.3f} m\n\n"
        
        # Add multi-level barycenter information
        info_text += f"\nMulti-Level Barycenters (P1 to P{self.num_barycenter_levels}):\n\n"
        for robot in self.robots_with_neighbors:
            neighbors_str = ", ".join(self.robot_neighbors[robot])
            info_text += f"{robot:8} + [{neighbors_str}]:\n"
            
            for level in range(1, self.num_barycenter_levels + 1):
                if level in self.barycenters[robot]:
                    bx, by = self.barycenters[robot][level]
                    level_name = f"P{level}"
                    info_text += f"  {level_name:3}: X={bx:8.3f} m, Y={by:8.3f} m\n"
            info_text += "\n"
        
        # Add swarm barycenter information
        if self.swarm_barycenter:
            sx, sy = self.swarm_barycenter
            info_text += f"Swarm Barycenter: X={sx:8.3f} m, Y={sy:8.3f} m\n"
        
        info_text += "\nDistances between robots:\n\n"
        
        # Format for distances
        distance_format = "{name1:8} - {name2:8}: {distance:8.3f} m\n"
        
        # Calculate distances between robots
        for i, name1 in enumerate(self.robot_names):
            for j, name2 in enumerate(self.robot_names):
                if i < j:  # Avoid duplicates
                    if self.current_positions[name1] and self.current_positions[name2]:
                        x1, y1 = self.current_positions[name1]
                        x2, y2 = self.current_positions[name2]
                        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        info_text += distance_format.format(name1=name1, name2=name2, distance=distance)
        
        # Update the text widget
        self.data_text.config(state=tk.NORMAL)
        self.data_text.delete(1.0, tk.END)
        self.data_text.insert(tk.END, info_text)
        self.data_text.config(state=tk.DISABLED)
    
    def toggle_robot_connections(self):
        """Toggle the display of lines connecting robots"""
        self.draw_robot_connections = not self.draw_robot_connections
        
        # Hide lines if disabled
        if not self.draw_robot_connections:
            for line in self.robot_connection_lines.values():
                line.set_visible(False)
            self.canvas.draw_idle()
        
        self.get_logger().info(f'Robot connections {"enabled" if self.draw_robot_connections else "disabled"}')
    
    def toggle_barycenters(self):
        """Toggle the display of barycenters"""
        self.draw_barycenters = not self.draw_barycenters
        
        # Update marker visibility
        self.update_barycenter_marker_visibility()
        
        # Hide/show swarm barycenter marker
        self.swarm_barycenter_marker.set_visible(self.draw_barycenters)
        
        self.canvas.draw_idle()
        self.get_logger().info(f'Barycenters {"enabled" if self.draw_barycenters else "disabled"}')
    
    def toggle_trajectories(self):
        """Toggle the display of robot trajectories"""
        self.draw_trajectories = not self.draw_trajectories
        
        # Hide/show trajectory lines
        for name in self.robot_names:
            self.trajectory_lines[name].set_visible(self.draw_trajectories)
        
        self.canvas.draw_idle()
        self.get_logger().info(f'Robot trajectories {"enabled" if self.draw_trajectories else "disabled"}')
    
    def clear_trajectories(self):
        """Clear all displayed trajectories"""
        # Reset trajectories
        for name in self.robot_names:
            self.trajectories[name] = {'x': [], 'y': [], 'time': []}
            self.trajectory_lines[name].set_data([], [])
        
        self.get_logger().info('Trajectories cleared')
        self.canvas.draw_idle()
    
    def on_closing(self):
        """Handle window closing"""
        self.get_logger().info('Closing visualization window')
        self.root.quit()
        self.root.destroy()
        rclpy.shutdown()
    
    def start(self):
        """Start the application"""
        # Create a thread for ROS spinning
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start the Tkinter main loop
        self.root.mainloop()
    
    def ros_spin(self):
        """Spin ROS in a separate thread"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    app = RobotVisualizerApp()
    
    try:
        app.start()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        app.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
