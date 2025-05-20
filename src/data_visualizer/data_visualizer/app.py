#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
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

# Maximum history for the scrolling distance plot
MAX_TIME_HISTORY = 60  # seconds

class RobotVisualizerApp(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # Configuration
        self.robot_names = ["Aramis", "Athos", "Porthos"]
        self.colors = ['red', 'green', 'blue']
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
        
        # Publishers for robot control
        self.cmd_vel_publishers = {}
        for i, topic in enumerate(self.cmd_vel_topics):
            self.cmd_vel_publishers[self.robot_names[i]] = self.create_publisher(
                Twist,
                topic,
                10
            )
        
        # Subscribe to pose topics
        for i, topic in enumerate(self.pose_topics):
            self.create_subscription(
                PoseStamped,
                topic,
                self.pose_callback_factory(self.robot_names[i]),
                qos_profile=self.qos_profile
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
    
    def setup_gui(self):
        # Create main window
        self.root = tk.Tk()
        self.root.title("Robot Position Visualizer")
        self.root.geometry("1200x900")  # Augmenter la hauteur
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
        # Left frame for the plot (using grid instead of pack)
        self.plot_frame = ttk.Frame(self.trajectory_tab)
        self.plot_frame.grid(row=0, column=0, sticky="nsew")
        self.plot_frame.rowconfigure(0, weight=1)
        self.plot_frame.rowconfigure(1, weight=0)
        self.plot_frame.rowconfigure(2, weight=0)
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

        # Robot control section
        self.robot_control_frame = ttk.LabelFrame(self.plot_frame, text="Robot Control")
        self.robot_control_frame.grid(row=2, column=0, sticky="ew", pady=5, padx=10)

        # Dropdown menu for selecting a robot
        ttk.Label(self.robot_control_frame, text="Robot:").grid(row=0, column=0, padx=5, pady=5)
        self.robot_var = tk.StringVar(value=self.robot_names[0])
        self.robot_dropdown = ttk.Combobox(
            self.robot_control_frame, 
            textvariable=self.robot_var, 
            values=self.robot_names,
            width=10
        )
        self.robot_dropdown.grid(row=0, column=1, padx=5, pady=5)
        
        # Text fields for target position (x, y) and angle
        ttk.Label(self.robot_control_frame, text="Target X:").grid(row=0, column=2, padx=5, pady=5)
        self.x_var = tk.DoubleVar(value=0.0)
        self.x_entry = ttk.Entry(self.robot_control_frame, textvariable=self.x_var, width=5)
        self.x_entry.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Label(self.robot_control_frame, text="Target Y:").grid(row=0, column=4, padx=5, pady=5)
        self.y_var = tk.DoubleVar(value=0.0)
        self.y_entry = ttk.Entry(self.robot_control_frame, textvariable=self.y_var, width=5)
        self.y_entry.grid(row=0, column=5, padx=5, pady=5)
        
        ttk.Label(self.robot_control_frame, text="Target Angle:").grid(row=0, column=6, padx=5, pady=5)
        self.z_var = tk.DoubleVar(value=0.0)
        self.z_entry = ttk.Entry(self.robot_control_frame, textvariable=self.z_var, width=5)
        self.z_entry.grid(row=0, column=7, padx=5, pady=5)
        
        # Button to send the robot to the specified position
        self.send_button = ttk.Button(
            self.robot_control_frame, 
            text="Go To Position", 
            command=self.send_robot_to_target
        )
        self.send_button.grid(row=0, column=8, padx=5, pady=5)
        
        # Button to stop robot
        self.stop_button = ttk.Button(
            self.robot_control_frame, 
            text="Stop", 
            command=self.stop_robot
        )
        self.stop_button.grid(row=0, column=9, padx=5, pady=5)
        
        # Create publishers for position control commands
        self.position_cmd_publishers = {}
        for name in self.robot_names:
            self.position_cmd_publishers[name] = self.create_publisher(
                PoseStamped,
                f"/{name}/target_pose",
                10
            )
    
    def send_robot_to_target(self):
        """Send the selected robot to the specified position using the robot_control node"""
        robot_name = self.robot_var.get()
        target_x = self.x_var.get()
        target_y = self.y_var.get()
        target_angle = self.z_var.get()
        
        # Create a target pose message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "world"
        
        # Set position
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = 0.0
        
        # Convert angle to quaternion (simple yaw rotation)
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(target_angle / 2.0)
        target_pose.pose.orientation.w = math.cos(target_angle / 2.0)
        
        # Publish the target pose
        if robot_name in self.position_cmd_publishers:
            self.position_cmd_publishers[robot_name].publish(target_pose)
            self.get_logger().info(f"Sent target position for {robot_name}: ({target_x}, {target_y}, {target_angle})")
            messagebox.showinfo("Command Sent", 
                f"{robot_name} is moving to position:\nX: {target_x} m\nY: {target_y} m\nAngle: {target_angle} rad")
        else:
            self.get_logger().error(f"Robot {robot_name} not found")
            messagebox.showerror("Error", f"Robot {robot_name} not found")
    
    def stop_robot(self):
        """Stop the selected robot by sending a zero velocity command"""
        robot_name = self.robot_var.get()
        
        if robot_name in self.cmd_vel_publishers:
            # Create a zero velocity command
            stop_cmd = Twist()
            self.cmd_vel_publishers[robot_name].publish(stop_cmd)
            
            self.get_logger().info(f"Stopping robot {robot_name}")
            messagebox.showinfo("Robot Stopped", f"{robot_name} has been stopped")
    
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
        self.ax.set_title('Robot Trajectories')
        
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
        self.orientation_arrows = {}  # Add dictionary for orientation arrows
        
        for i, name in enumerate(self.robot_names):
            color = self.colors[i % len(self.colors)]
            
            # Line for the trajectory
            line, = self.ax.plot([], [], '-', color=color, label=f'{name}', linewidth=2, alpha=0.7)
            self.trajectory_lines[name] = line
            
            # Marker for the current position
            marker, = self.ax.plot([], [], 'o', color=color, markersize=10)
            self.position_markers[name] = marker
            
            # Arrow for orientation (initially with length 0)
            arrow = self.ax.quiver(0, 0, 0, 0, color=color, scale=1, scale_units='inches', width=0.03)
            self.orientation_arrows[name] = arrow
        
        # Create connection lines between robots (initially invisible)
        for i, name1 in enumerate(self.robot_names):
            for j, name2 in enumerate(self.robot_names):
                if i < j:  # Avoid duplicates
                    line, = self.ax.plot([], [], 'k--', linewidth=1, alpha=0.5, visible=False)
                    self.robot_connection_lines[f"{name1}-{name2}"] = line
        
        # Add legend
        self.ax.legend(loc='upper right')
    
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
                if x_data and y_data:
                    self.trajectory_lines[name].set_data(y_data, x_data)
                    plot_needs_update = True
                    
                    # Update position marker
                    if self.current_positions[name]:
                        x_pos, y_pos = self.current_positions[name]
                        self.position_markers[name].set_data([y_pos], [x_pos])
                        
                        # Update orientation arrow if we have angle data
                        if self.current_angles[name] is not None:
                            # Calculate arrow vector components (adjust for the plot's axes)
                            # Since x is plotted on y-axis and y on x-axis (inverted), we need to switch components
                            arrow_length = 0.2  # Length of the orientation arrow
                            dx = arrow_length * math.sin(self.current_angles[name])
                            dy = arrow_length * math.cos(self.current_angles[name])
                            
                            # Remove old arrow and create new one at the current position
                            self.orientation_arrows[name].remove()
                            self.orientation_arrows[name] = self.ax.quiver(
                                y_pos, x_pos, dx, dy, 
                                color=self.trajectory_lines[name].get_color(),
                                scale=1, scale_units='inches', width=0.03
                            )
            
            # Update connection lines if enabled
            if self.draw_robot_connections:
                for i, name1 in enumerate(self.robot_names):
                    for j, name2 in enumerate(self.robot_names):
                        if i < j:  # Avoid duplicates
                            if (self.current_positions[name1] and self.current_positions[name2]):
                                x1, y1 = self.current_positions[name1]
                                x2, y2 = self.current_positions[name2]
                                line = self.robot_connection_lines[f"{name1}-{name2}"]
                                line.set_data([y1, y2], [x1, x2])
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
        """Update the position and distance information in the text area"""
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
            
        # Build text with position information
        info_text = "Robot positions:\n\n"
        
        # Format for positions including yaw
        position_format = "{name:8}: X={x:8.3f} m, Y={y:8.3f} m, Yaw={yaw:8.2f} deg\n"
        
        for name in self.robot_names:
            if self.current_positions[name] and self.current_angles[name] is not None:
                x, y = self.current_positions[name]
                yaw_deg = math.degrees(self.current_angles[name])
                info_text += position_format.format(name=name, x=x, y=y, yaw=yaw_deg)
        
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
    
    def send_robot_to_position(self):
        """Open dialog to send a robot to a position"""
        # Create dialog
        robot_dialog = RobotDestinationDialog(self.root, self.robot_names)
        if robot_dialog.result:
            robot_name, x, y = robot_dialog.result
            self.get_logger().info(f"Sending {robot_name} to position ({x}, {y})")
            
            # Send command to robot - in a real implementation we'd use an action client
            # For now, we'll just display a message
            messagebox.showinfo("Command Sent", 
                               f"Command to send {robot_name} to ({x:.2f}, {y:.2f}) has been sent!")
    
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


class RobotDestinationDialog:
    """Dialog for selecting a robot and destination"""
    def __init__(self, parent, robot_names):
        self.result = None
        
        # Create dialog window
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("Send Robot to Position")
        self.dialog.geometry("300x200")
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Create widgets
        ttk.Label(self.dialog, text="Select Robot:").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.robot_var = tk.StringVar(value=robot_names[0])
        self.robot_combo = ttk.Combobox(self.dialog, textvariable=self.robot_var, values=robot_names)
        self.robot_combo.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        
        ttk.Label(self.dialog, text="X Position:").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        self.x_var = tk.DoubleVar(value=0.0)
        self.x_entry = ttk.Entry(self.dialog, textvariable=self.x_var)
        self.x_entry.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        
        ttk.Label(self.dialog, text="Y Position:").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        self.y_var = tk.DoubleVar(value=0.0)
        self.y_entry = ttk.Entry(self.dialog, textvariable=self.y_var)
        self.y_entry.grid(row=2, column=1, padx=10, pady=10, sticky="ew")
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.grid(row=3, column=0, columnspan=2, pady=20)
        
        ttk.Button(button_frame, text="Send", command=self.on_send).pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="Cancel", command=self.on_cancel).pack(side=tk.LEFT, padx=10)
        
        # Configure grid
        self.dialog.columnconfigure(1, weight=1)
        
        # Wait for dialog to close
        parent.wait_window(self.dialog)
    
    def on_send(self):
        try:
            robot = self.robot_var.get()
            x = self.x_var.get()
            y = self.y_var.get()
            self.result = (robot, x, y)
            self.dialog.destroy()
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid position values: {e}")
    
    def on_cancel(self):
        self.dialog.destroy()


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
