import tkinter as tk
import math

class TextDisplay:
    def __init__(self, config, data_manager, text_widget):
        self.config = config
        self.data_manager = data_manager
        self.text_widget = text_widget
    
    def update_position_info(self):
        """Update the position and velocity information in the text area"""
        if not all(self.data_manager.current_positions.values()):
            return
        
        # Build text with position and velocity information
        info_text = "Robot Data:\n\n"
        
        # Format for positions including yaw and velocity
        robot_format = "{name:8}:\n  Position: X={x:8.3f} m, Y={y:8.3f} m\n  Yaw:      {yaw:8.2f} deg\n  Velocity: Vx={vx:7.3f} m/s, Vy={vy:7.3f} m/s\n\n"
        
        for name in self.config.robot_names:
            if self.data_manager.current_positions[name] and self.data_manager.current_angles[name] is not None:
                x, y = self.data_manager.current_positions[name]
                yaw_deg = math.degrees(self.data_manager.current_angles[name])
                
                # Get velocity data
                vx, vy = (0.0, 0.0)
                if self.data_manager.current_velocities[name] is not None:
                    vx, vy = self.data_manager.current_velocities[name]
                
                info_text += robot_format.format(
                    name=name, x=x, y=y, yaw=yaw_deg, vx=vx, vy=vy
                )
        
        # Add goal point information
        if self.data_manager.goal_point:
            goal_x, goal_y = self.data_manager.goal_point
            info_text += f"Goal Point: X={goal_x:8.3f} m, Y={goal_y:8.3f} m\n\n"
        
        # Add multi-level barycenter information
        info_text += self.format_barycenter_info()
        
        # Add distance information
        info_text += self.format_distance_info()
        
        # Update the text widget
        self.text_widget.config(state=tk.NORMAL)
        self.text_widget.delete(1.0, tk.END)
        self.text_widget.insert(tk.END, info_text)
        self.text_widget.config(state=tk.DISABLED)
    
    def format_barycenter_info(self):
        """Format barycenter information"""
        # Get the current barycenter levels from the GUI manager
        num_levels = getattr(self, 'gui_manager', None)
        if num_levels:
            num_levels = self.gui_manager.num_barycenter_levels
        else:
            num_levels = 3  # Default fallback
        
        info_text = f"\nMulti-Level Barycenters (P1 to P{num_levels}):\n\n"
        for robot in self.config.robots_with_neighbors:
            neighbors_str = ", ".join(self.config.robot_neighbors[robot])
            info_text += f"{robot:8} + [{neighbors_str}]:\n"
            
            for level in range(1, num_levels + 1):
                if level in self.data_manager.barycenters[robot]:
                    bx, by = self.data_manager.barycenters[robot][level]
                    level_name = f"P{level}"
                    info_text += f"  {level_name:3}: X={bx:8.3f} m, Y={by:8.3f} m\n"
            info_text += "\n"
        
        # Add swarm barycenter information
        if self.data_manager.swarm_barycenter:
            sx, sy = self.data_manager.swarm_barycenter
            info_text += f"Swarm Barycenter: X={sx:8.3f} m, Y={sy:8.3f} m\n"
        
        return info_text
    
    def format_distance_info(self):
        """Format distance information"""
        info_text = "\nDistances between robots:\n\n"
        
        # Format for distances
        distance_format = "{name1:8} - {name2:8}: {distance:8.3f} m\n"
        
        # Calculate distances between robots
        for i, name1 in enumerate(self.config.robot_names):
            for j, name2 in enumerate(self.config.robot_names):
                if i < j:  # Avoid duplicates
                    if (self.data_manager.current_positions[name1] and 
                        self.data_manager.current_positions[name2]):
                        x1, y1 = self.data_manager.current_positions[name1]
                        x2, y2 = self.data_manager.current_positions[name2]
                        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        info_text += distance_format.format(name1=name1, name2=name2, distance=distance)
        
        return info_text
    
    def set_barycenter_levels(self, levels):
        """Set the number of barycenter levels for display"""
        self.num_barycenter_levels = levels
    
    def set_gui_manager_reference(self, gui_manager):
        """Set a reference to the GUI manager to access current settings"""
        self.gui_manager = gui_manager
