import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import math
from .plot_manager import PlotManager
from .text_display import TextDisplay
from .estimation_plot_manager import EstimationPlotManager

class GUIManager:
    def __init__(self, config, data_manager):
        self.config = config
        self.data_manager = data_manager
        self.text_update_needed = True
        
        # Flags for drawing options
        self.draw_robot_connections = False
        self.draw_trajectories = True
        self.draw_barycenters = True
        self.num_barycenter_levels = 3
        self.max_barycenter_levels = 10
        
        # Initialize estimation plot manager first
        self.estimation_plot_manager = EstimationPlotManager(config, data_manager)
        
        # Initialize GUI
        self.setup_gui()
        
        # Initialize plot manager
        self.plot_manager = PlotManager(config, data_manager, self)
        
        # Initialize text display
        self.text_display = TextDisplay(config, data_manager, self.data_text)
        # Set reference to this GUI manager for accessing current settings
        self.text_display.set_gui_manager_reference(self)
    
    def setup_gui(self):
        """Create and setup the GUI"""
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
        self.main_frame.columnconfigure(0, weight=3)
        self.main_frame.columnconfigure(1, weight=1)
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
        
        # Create estimation tab
        self.estimation_plot_manager.create_tab(self.notebook)
        
        # Right frame for the data display
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

        # Create control buttons
        self.setup_control_buttons()
    
    def setup_control_buttons(self):
        """Setup control buttons"""
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
    
    def update_plot(self):
        """Update all plots and displays"""
        try:
            # Update the main plot
            plot_updated = self.plot_manager.update_trajectory_plot()
            
            # Update distance data and plot
            self.data_manager.update_distance_data()
            if self.notebook.index(self.notebook.select()) == 1:  # Distance tab is selected
                self.plot_manager.update_distance_plot()
            
            # Update estimation plot
            self.estimation_plot_manager.update_plot()
            
            # Update text display if needed
            if self.text_update_needed and self.data_manager.positions_changed():
                self.text_display.update_position_info()
                self.text_update_needed = False
                
        except Exception as e:
            print(f'Error updating plot: {e}')
    
    def mark_text_update_needed(self):
        """Mark that text update is needed"""
        self.text_update_needed = True
    
    def clear_trajectories(self):
        """Clear all displayed trajectories"""
        self.data_manager.clear_trajectories()
        self.plot_manager.clear_trajectory_display()
        print('Trajectories cleared')
    
    def toggle_robot_connections(self):
        """Toggle the display of lines connecting robots"""
        self.draw_robot_connections = not self.draw_robot_connections
        self.plot_manager.update_connection_visibility()
        print(f'Robot connections {"enabled" if self.draw_robot_connections else "disabled"}')
    
    def toggle_barycenters(self):
        """Toggle the display of barycenters"""
        self.draw_barycenters = not self.draw_barycenters
        self.plot_manager.update_barycenter_visibility()
        print(f'Barycenters {"enabled" if self.draw_barycenters else "disabled"}')
    
    def toggle_trajectories(self):
        """Toggle the display of robot trajectories"""
        self.draw_trajectories = not self.draw_trajectories
        self.plot_manager.update_trajectory_visibility()
        print(f'Robot trajectories {"enabled" if self.draw_trajectories else "disabled"}')
    
    def update_barycenter_levels(self):
        """Update the number of barycenter levels to calculate"""
        self.num_barycenter_levels = self.level_var.get()
        print(f'Barycenter levels set to: {self.num_barycenter_levels}')
        self.plot_manager.update_barycenter_visibility()
    
    def on_level_change(self, *args):
        """Handle level variable change"""
        self.update_barycenter_levels()
    
    def on_closing(self):
        """Handle window closing"""
        print('Closing visualization window')
        self.root.quit()
        self.root.destroy()
        import rclpy
        rclpy.shutdown()
    
    def start(self):
        """Start the GUI main loop"""
        # Initialize the plot manager
        self.plot_manager.setup_plots(self.ax, self.distance_ax, self.canvas, self.distance_canvas)
        
        # Start the Tkinter main loop
        self.root.mainloop()
