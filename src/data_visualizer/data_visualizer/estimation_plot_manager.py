import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

class EstimationPlotManager:
    def __init__(self, config, data_manager):
        self.config = config
        self.data_manager = data_manager
        
        # Plot elements
        self.real_position_markers = {}
        self.estimated_position_markers = {}
        self.estimation_lines = {}
        
    def create_tab(self, notebook):
        """Create the estimation tab in the notebook"""
        self.tab_frame = ttk.Frame(notebook)
        notebook.add(self.tab_frame, text="Positions Estimées")
        
        # Create main container
        main_container = ttk.PanedWindow(self.tab_frame, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left side: Plot
        plot_frame = ttk.Frame(main_container)
        main_container.add(plot_frame, weight=3)
        
        # Create matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Right side: Information panel
        info_frame = ttk.Frame(main_container)
        main_container.add(info_frame, weight=1)
        
        # Create text display for positions
        self.create_position_display(info_frame)
        
        # Setup the plot
        self.setup_plot()
        
        return self.tab_frame
    
    def create_position_display(self, parent):
        """Create text display for position information"""
        # Title
        title_label = ttk.Label(parent, text="Positions Estimées", font=("Arial", 12, "bold"))
        title_label.pack(pady=(0, 10))
        
        # Scrollable text widget
        text_frame = ttk.Frame(parent)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        # Text widget with scrollbar
        self.position_text = tk.Text(text_frame, wrap=tk.WORD, state=tk.DISABLED, 
                                   width=30, font=("Courier", 9))
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.position_text.yview)
        self.position_text.configure(yscrollcommand=scrollbar.set)
        
        self.position_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    def setup_plot(self):
        """Setup the initial plot"""
        self.ax.set_xlabel('Y Position (m)')
        self.ax.set_ylabel('X Position (m)')
        self.ax.set_title('Positions Réelles vs Estimées')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(2, -2)
        self.ax.set_ylim(-2, 2)
        
        # Create plot elements for each robot
        for i, robot in enumerate(self.config.robot_names):
            color = self.config.colors[i % len(self.config.colors)]
            
            # Real position marker (filled circle)
            real_marker, = self.ax.plot([], [], 'o', color=color, markersize=10, 
                                      label=f'{robot} (réel)', alpha=0.8)
            self.real_position_markers[robot] = real_marker
            
            # Initialize estimated position markers for this robot's neighbors
            self.estimated_position_markers[robot] = {}
            self.estimation_lines[robot] = {}
            
            if robot in self.config.robot_neighbors:
                for j, neighbor in enumerate(self.config.robot_neighbors[robot]):
                    # Estimated position marker (empty circle with border)
                    est_marker, = self.ax.plot([], [], 'o', color=color, markersize=8, 
                                             fillstyle='none', markeredgewidth=2,
                                             label=f'{neighbor} estimé par {robot}' if i == 0 else "")
                    self.estimated_position_markers[robot][neighbor] = est_marker
                    
                    # Line connecting real and estimated positions
                    line, = self.ax.plot([], [], '--', color=color, alpha=0.6, linewidth=1)
                    self.estimation_lines[robot][neighbor] = line
        
        # Add legend
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.fig.tight_layout()
    
    def update_plot(self):
        """Update the plot with current data"""
        plot_needs_update = False
        
        # Update real positions
        for robot in self.config.robot_names:
            if self.data_manager.current_positions[robot]:
                x_pos, y_pos = self.data_manager.current_positions[robot]
                self.real_position_markers[robot].set_data([y_pos], [x_pos])
                plot_needs_update = True
        
        # Update estimated positions and connection lines
        for robot in self.config.robot_names:
            if robot in self.config.robot_neighbors:
                for neighbor in self.config.robot_neighbors[robot]:
                    # Update estimated position marker
                    if self.data_manager.estimated_positions[robot][neighbor]:
                        est_x, est_y = self.data_manager.estimated_positions[robot][neighbor]
                        self.estimated_position_markers[robot][neighbor].set_data([est_y], [est_x])
                        
                        # Update connection line between real and estimated positions
                        if self.data_manager.current_positions[neighbor]:
                            real_x, real_y = self.data_manager.current_positions[neighbor]
                            self.estimation_lines[robot][neighbor].set_data([real_y, est_y], [real_x, est_x])
                        
                        plot_needs_update = True
        
        if plot_needs_update:
            self.ax.set_xlim(2, -2)
            self.ax.set_ylim(-2, 2)
            self.canvas.draw_idle()
        
        # Update text display
        self.update_position_text()
    
    def update_position_text(self):
        """Update the text display with position information"""
        # Save current scroll position
        current_position = self.position_text.yview()
        
        self.position_text.config(state=tk.NORMAL)
        self.position_text.delete(1.0, tk.END)
        
        for robot in self.config.robot_names:
            self.position_text.insert(tk.END, f"=== {robot} ===\n", "header")
            
            # Real position
            if self.data_manager.current_positions[robot]:
                real_x, real_y = self.data_manager.current_positions[robot]
                self.position_text.insert(tk.END, f"Position réelle:\n")
                self.position_text.insert(tk.END, f"  X: {real_x:.3f} m\n")
                self.position_text.insert(tk.END, f"  Y: {real_y:.3f} m\n")
                
                # Show estimated positions right after real position for comparison
                if robot in self.config.robot_neighbors:
                    self.position_text.insert(tk.END, f"\nPositions estimées par {robot}:\n")
                    for neighbor in self.config.robot_neighbors[robot]:
                        if self.data_manager.estimated_positions[robot][neighbor]:
                            est_x, est_y = self.data_manager.estimated_positions[robot][neighbor]
                            self.position_text.insert(tk.END, f"  {neighbor} (estimé):\n")
                            self.position_text.insert(tk.END, f"    X: {est_x:.3f} m\n")
                            self.position_text.insert(tk.END, f"    Y: {est_y:.3f} m\n")
                            
                            # Calculate estimation error if real position is available
                            if self.data_manager.current_positions[neighbor]:
                                real_neighbor_x, real_neighbor_y = self.data_manager.current_positions[neighbor]
                                error = math.sqrt((est_x - real_neighbor_x)**2 + (est_y - real_neighbor_y)**2)
                                self.position_text.insert(tk.END, f"    Erreur: {error:.3f} m\n")
                        else:
                            self.position_text.insert(tk.END, f"  {neighbor}: Non disponible\n")
                
                # Show who estimates this robot's position
                estimators = []
                for other_robot in self.config.robot_names:
                    if (other_robot != robot and 
                        other_robot in self.config.robot_neighbors and 
                        robot in self.config.robot_neighbors[other_robot]):
                        estimators.append(other_robot)
                
                if estimators:
                    self.position_text.insert(tk.END, f"\nPosition de {robot} estimée par:\n")
                    for estimator in estimators:
                        if self.data_manager.estimated_positions[estimator][robot]:
                            est_x, est_y = self.data_manager.estimated_positions[estimator][robot]
                            self.position_text.insert(tk.END, f"  {estimator}:\n")
                            self.position_text.insert(tk.END, f"    X: {est_x:.3f} m\n")
                            self.position_text.insert(tk.END, f"    Y: {est_y:.3f} m\n")
                            
                            # Calculate estimation error
                            error = math.sqrt((est_x - real_x)**2 + (est_y - real_y)**2)
                            self.position_text.insert(tk.END, f"    Erreur: {error:.3f} m\n")
                        else:
                            self.position_text.insert(tk.END, f"  {estimator}: Non disponible\n")
            else:
                self.position_text.insert(tk.END, f"Position réelle: Non disponible\n")
            
            self.position_text.insert(tk.END, "\n" + "="*30 + "\n\n")
        
        self.position_text.config(state=tk.DISABLED)
        
        # Restore scroll position
        self.position_text.yview_moveto(current_position[0])
