import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import csv
from dataclasses import dataclass
from typing import Tuple, List, Dict
import json

try:
    import ezdxf
    DXF_AVAILABLE = True
except ImportError:
    DXF_AVAILABLE = False
    print("Warning: ezdxf not installed. DXF export will be disabled.")
    print("Install with: pip install ezdxf")


@dataclass
class CamParameters:
    """Container for cam design parameters"""
    stroke: float = 30.0  # mm
    base_circle_radius: float = 50.0  # mm
    rise_angle: float = 120.0  # degrees
    dwell1_angle: float = 30.0  # degrees
    return_angle: float = 120.0  # degrees
    dwell2_angle: float = 30.0  # degrees
    motion_law_rise: str = 'SHM'
    motion_law_return: str = 'SHM'
    resolution: int = 360  # points per revolution


# ==================== MEMBER 1: MOTION LAWS ====================
class MotionLaws:
    """
    Implementation of standard cam motion laws
    Each method returns (displacement, velocity, acceleration)
    """
    
    @staticmethod
    def simple_harmonic_motion(theta: np.ndarray, L: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simple Harmonic Motion (SHM)
        theta: normalized angle (0 to 1)
        L: total lift/stroke
        """
        s = L * (1 - np.cos(np.pi * theta)) / 2
        v = (np.pi * L / 2) * np.sin(np.pi * theta)
        a = (np.pi**2 * L / 2) * np.cos(np.pi * theta)
        return s, v, a
    
    @staticmethod
    def uniform_velocity(theta: np.ndarray, L: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Uniform Velocity (constant velocity motion)
        Simplified version with linear displacement
        """
        s = L * theta
        v = np.ones_like(theta) * L
        a = np.zeros_like(theta)
        return s, v, a
    
    @staticmethod
    def uniform_acceleration(theta: np.ndarray, L: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Uniform Acceleration (Parabolic motion)
        Acceleration in first half, deceleration in second half
        """
        s = np.zeros_like(theta)
        v = np.zeros_like(theta)
        a = np.zeros_like(theta)
        
        # First half: acceleration
        mask1 = theta <= 0.5
        s[mask1] = 2 * L * theta[mask1]**2
        v[mask1] = 4 * L * theta[mask1]
        a[mask1] = 4 * L
        
        # Second half: deceleration
        mask2 = theta > 0.5
        s[mask2] = L * (1 - 2 * (1 - theta[mask2])**2)
        v[mask2] = 4 * L * (1 - theta[mask2])
        a[mask2] = -4 * L
        
        return s, v, a
    
    @staticmethod
    def cycloidal(theta: np.ndarray, L: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Cycloidal Motion (smoothest motion, good for high-speed cams)
        """
        s = L * (theta - np.sin(2 * np.pi * theta) / (2 * np.pi))
        v = L * (1 - np.cos(2 * np.pi * theta))
        a = 2 * np.pi * L * np.sin(2 * np.pi * theta)
        return s, v, a
    
    @staticmethod
    def get_motion_law(name: str):
        """Return the appropriate motion law function"""
        laws = {
            'SHM': MotionLaws.simple_harmonic_motion,
            'Uniform Velocity': MotionLaws.uniform_velocity,
            'Uniform Acceleration': MotionLaws.uniform_acceleration,
            'Cycloidal': MotionLaws.cycloidal
        }
        return laws.get(name, MotionLaws.simple_harmonic_motion)


# ==================== MEMBER 2: GEOMETRIC ROUTINES ====================
class CamGeometry:
    """
    Geometric calculations for cam profile generation
    Handles pitch curve and profile generation for knife-edge follower
    """
    
    def __init__(self, params: CamParameters):
        self.params = params
        self.cam_angles = None
        self.displacement = None
        self.velocity = None
        self.acceleration = None
        self.profile_x = None
        self.profile_y = None
        
    def calculate_displacement_diagram(self):
        """Calculate complete displacement, velocity, and acceleration diagrams"""
        # Convert angles to radians
        rise_rad = np.radians(self.params.rise_angle)
        dwell1_rad = np.radians(self.params.dwell1_angle)
        return_rad = np.radians(self.params.return_angle)
        dwell2_rad = np.radians(self.params.dwell2_angle)
        total_angle = rise_rad + dwell1_rad + return_rad + dwell2_rad
        
        # Create angle array
        n_points = self.params.resolution
        self.cam_angles = np.linspace(0, 2*np.pi, n_points)
        
        # Initialize arrays
        self.displacement = np.zeros(n_points)
        self.velocity = np.zeros(n_points)
        self.acceleration = np.zeros(n_points)
        
        # Get motion law functions
        rise_law = MotionLaws.get_motion_law(self.params.motion_law_rise)
        return_law = MotionLaws.get_motion_law(self.params.motion_law_return)
        
        for i, angle in enumerate(self.cam_angles):
            if angle < rise_rad:
                # Rise segment
                theta_norm = angle / rise_rad
                s, v, a = rise_law(np.array([theta_norm]), self.params.stroke)
                self.displacement[i] = s[0]
                self.velocity[i] = v[0] / rise_rad
                self.acceleration[i] = a[0] / (rise_rad**2)
                
            elif angle < rise_rad + dwell1_rad:
                # First dwell
                self.displacement[i] = self.params.stroke
                self.velocity[i] = 0
                self.acceleration[i] = 0
                
            elif angle < rise_rad + dwell1_rad + return_rad:
                # Return segment
                theta_norm = (angle - rise_rad - dwell1_rad) / return_rad
                s, v, a = return_law(np.array([theta_norm]), self.params.stroke)
                self.displacement[i] = self.params.stroke - s[0]
                self.velocity[i] = -v[0] / return_rad
                self.acceleration[i] = -a[0] / (return_rad**2)
                
            else:
                # Second dwell
                self.displacement[i] = 0
                self.velocity[i] = 0
                self.acceleration[i] = 0
    
    def calculate_pitch_curve(self):
        """
        Calculate pitch curve (for knife-edge follower, this is the cam profile)
        For knife-edge follower: radial distance = base circle radius + displacement
        """
        if self.displacement is None:
            self.calculate_displacement_diagram()
        
        # Calculate radial distances
        radii = self.params.base_circle_radius + self.displacement
        
        # Convert to Cartesian coordinates
        self.profile_x = radii * np.cos(self.cam_angles)
        self.profile_y = radii * np.sin(self.cam_angles)
    
    def get_profile_coordinates(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return cam profile coordinates"""
        if self.profile_x is None:
            self.calculate_pitch_curve()
        return self.profile_x, self.profile_y
    
    def get_displacement_data(self) -> Dict[str, np.ndarray]:
        """Return displacement diagram data"""
        if self.displacement is None:
            self.calculate_displacement_diagram()
        
        return {
            'angles_deg': np.degrees(self.cam_angles),
            'displacement': self.displacement,
            'velocity': self.velocity,
            'acceleration': self.acceleration
        }


# ==================== MEMBER 3: FILE EXPORT ====================
class CamExporter:
    """
    Export cam profiles to CSV and DXF formats
    """
    
    @staticmethod
    def export_to_csv(filename: str, x: np.ndarray, y: np.ndarray, 
                      displacement_data: Dict = None):
        """Export cam profile coordinates to CSV"""
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                writer.writerow(['Point', 'X (mm)', 'Y (mm)', 'Angle (deg)', 'Radius (mm)'])
                
                # Write coordinates
                for i, (xi, yi) in enumerate(zip(x, y)):
                    radius = np.sqrt(xi**2 + yi**2)
                    angle = np.degrees(np.arctan2(yi, xi))
                    writer.writerow([i+1, f'{xi:.4f}', f'{yi:.4f}', f'{angle:.2f}', f'{radius:.4f}'])
                
                # Write displacement data if provided
                if displacement_data:
                    writer.writerow([])
                    writer.writerow(['Displacement Diagram Data'])
                    writer.writerow(['Angle (deg)', 'Displacement (mm)', 'Velocity (mm/rad)', 'Acceleration (mm/rad²)'])
                    
                    angles = displacement_data['angles_deg']
                    disp = displacement_data['displacement']
                    vel = displacement_data['velocity']
                    acc = displacement_data['acceleration']
                    
                    for ang, d, v, a in zip(angles, disp, vel, acc):
                        writer.writerow([f'{ang:.2f}', f'{d:.4f}', f'{v:.4f}', f'{a:.4f}'])
            
            return True
        except Exception as e:
            print(f"Error exporting to CSV: {e}")
            return False
    
    @staticmethod
    def export_to_dxf(filename: str, x: np.ndarray, y: np.ndarray, params: CamParameters):
        """Export cam profile to DXF format"""
        if not DXF_AVAILABLE:
            print("DXF export not available. Install ezdxf: pip install ezdxf")
            return False
        
        try:
            # Create new DXF document
            doc = ezdxf.new('R2010')
            msp = doc.modelspace()
            
            # Create polyline for cam profile
            points = [(xi, yi) for xi, yi in zip(x, y)]
            msp.add_lwpolyline(points, close=True)
            
            # Add base circle for reference
            msp.add_circle((0, 0), params.base_circle_radius, 
                          dxfattribs={'color': 2, 'layer': 'BASE_CIRCLE'})
            
            # Add prime circle (base + stroke)
            msp.add_circle((0, 0), params.base_circle_radius + params.stroke,
                          dxfattribs={'color': 3, 'layer': 'PRIME_CIRCLE'})
            
            # Add center point
            msp.add_point((0, 0), dxfattribs={'color': 1})
            
            # Save DXF file
            doc.saveas(filename)
            return True
            
        except Exception as e:
            print(f"Error exporting to DXF: {e}")
            return False
    
    @staticmethod
    def export_parameters(filename: str, params: CamParameters):
        """Export cam parameters to JSON file"""
        try:
            param_dict = {
                'stroke': params.stroke,
                'base_circle_radius': params.base_circle_radius,
                'rise_angle': params.rise_angle,
                'dwell1_angle': params.dwell1_angle,
                'return_angle': params.return_angle,
                'dwell2_angle': params.dwell2_angle,
                'motion_law_rise': params.motion_law_rise,
                'motion_law_return': params.motion_law_return,
                'resolution': params.resolution
            }
            
            with open(filename, 'w') as f:
                json.dump(param_dict, f, indent=4)
            
            return True
        except Exception as e:
            print(f"Error exporting parameters: {e}")
            return False


# ==================== MEMBER 4: GUI AND PLOTTING ====================
class CamGUI:
    """
    Graphical User Interface for cam profile generator
    Includes plotting and user controls
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("Cam Profile Generator - Knife-Edge Follower")
        self.root.geometry("1600x1000")  # Larger window
        
        # Set minimum window size
        self.root.minsize(1400, 900)
        
        self.params = CamParameters()
        self.cam_geometry = None
        
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the GUI layout"""
        # Create main frames with larger padding
        control_frame = ttk.Frame(self.root, padding="20")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        plot_frame = ttk.Frame(self.root, padding="20")
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Setup controls
        self.setup_controls(control_frame)
        
        # Setup plot area
        self.setup_plots(plot_frame)
        
    def setup_controls(self, parent):
        """Setup input controls"""
        # Configure style for larger fonts
        style = ttk.Style()
        style.configure('Large.TLabel', font=('Arial', 15))
        style.configure('Large.TEntry', font=('Arial', 15))
        style.configure('Large.TButton', font=('Arial', 15, 'bold'), padding=10)
        style.configure('Title.TLabel', font=('Arial', 18, 'bold'))
        style.configure('Section.TLabel', font=('Arial', 15, 'bold'))
        
        # Title
        title = ttk.Label(parent, text="Cam Design Parameters", 
                         style='Title.TLabel')
        title.grid(row=0, column=0, columnspan=2, pady=15)
        
        # Input fields
        row = 1
        
        # Stroke
        ttk.Label(parent, text="Stroke (mm):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.stroke_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.stroke_entry.insert(0, str(self.params.stroke))
        self.stroke_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Base Circle Radius
        ttk.Label(parent, text="Base Circle Radius (mm):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.base_radius_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.base_radius_entry.insert(0, str(self.params.base_circle_radius))
        self.base_radius_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Rise Angle
        ttk.Label(parent, text="Rise Angle (deg):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.rise_angle_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.rise_angle_entry.insert(0, str(self.params.rise_angle))
        self.rise_angle_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Dwell 1 Angle
        ttk.Label(parent, text="Dwell 1 Angle (deg):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.dwell1_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.dwell1_entry.insert(0, str(self.params.dwell1_angle))
        self.dwell1_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Return Angle
        ttk.Label(parent, text="Return Angle (deg):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.return_angle_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.return_angle_entry.insert(0, str(self.params.return_angle))
        self.return_angle_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Dwell 2 Angle
        ttk.Label(parent, text="Dwell 2 Angle (deg):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.dwell2_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.dwell2_entry.insert(0, str(self.params.dwell2_angle))
        self.dwell2_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Motion Law - Rise
        ttk.Label(parent, text="Motion Law (Rise):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.rise_law_combo = ttk.Combobox(parent, width=18, font=('Arial', 15),
                                           values=['SHM', 'Uniform Velocity', 
                                                  'Uniform Acceleration', 'Cycloidal'])
        self.rise_law_combo.set(self.params.motion_law_rise)
        self.rise_law_combo.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Motion Law - Return
        ttk.Label(parent, text="Motion Law (Return):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.return_law_combo = ttk.Combobox(parent, width=18, font=('Arial', 15),
                                            values=['SHM', 'Uniform Velocity',
                                                   'Uniform Acceleration', 'Cycloidal'])
        self.return_law_combo.set(self.params.motion_law_return)
        self.return_law_combo.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Resolution
        ttk.Label(parent, text="Resolution (points):", style='Large.TLabel').grid(row=row, column=0, sticky=tk.W, pady=8)
        self.resolution_entry = ttk.Entry(parent, width=20, font=('Arial', 15))
        self.resolution_entry.insert(0, str(self.params.resolution))
        self.resolution_entry.grid(row=row, column=1, pady=8, padx=5)
        row += 1
        
        # Buttons
        generate_btn = ttk.Button(parent, text="Generate Cam Profile", 
                  command=self.generate_profile, style='Large.TButton')
        generate_btn.grid(row=row, column=0, columnspan=2, pady=25, sticky=(tk.W, tk.E), padx=10)
        row += 1
        
        # Export section
        ttk.Label(parent, text="Export:", style='Section.TLabel').grid(row=row, column=0, columnspan=2, pady=(15, 10))
        row += 1
        
        csv_btn = ttk.Button(parent, text="Export to CSV", 
                  command=self.export_csv, style='Large.TButton')
        csv_btn.grid(row=row, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E), padx=10)
        row += 1
        
        if DXF_AVAILABLE:
            dxf_btn = ttk.Button(parent, text="Export to DXF", 
                      command=self.export_dxf, style='Large.TButton')
            dxf_btn.grid(row=row, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E), padx=10)
        else:
            ttk.Label(parent, text="DXF export unavailable\n(pip install ezdxf)",
                     foreground='red', font=('Arial', 10)).grid(row=row, column=0, columnspan=2, pady=8)
        row += 1
        
        params_btn = ttk.Button(parent, text="Save Parameters", 
                  command=self.save_parameters, style='Large.TButton')
        params_btn.grid(row=row, column=0, columnspan=2, pady=8, sticky=(tk.W, tk.E), padx=10)
        
    def setup_plots(self, parent):
        """Setup matplotlib plot areas"""
        # Create notebook for tabs with larger font
        style = ttk.Style()
        style.configure('TNotebook.Tab', font=('Arial', 11), padding=[20, 10])
        
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Create tabs
        self.disp_frame = ttk.Frame(self.notebook)
        self.vel_frame = ttk.Frame(self.notebook)
        self.acc_frame = ttk.Frame(self.notebook)
        self.profile_frame = ttk.Frame(self.notebook)
        
        self.notebook.add(self.disp_frame, text='  Displacement  ')
        self.notebook.add(self.vel_frame, text='  Velocity  ')
        self.notebook.add(self.acc_frame, text='  Acceleration  ')
        self.notebook.add(self.profile_frame, text='  Cam Profile  ')
        
        # Initialize plots
        self.init_plots()
        
    def init_plots(self):
        """Initialize empty plots"""
        # Increase default font sizes for plots
        plt.rcParams.update({
            'font.size': 11,
            'axes.labelsize': 12,
            'axes.titlesize': 14,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'legend.fontsize': 11
        })
        
        # Displacement plot
        self.fig_disp, self.ax_disp = plt.subplots(figsize=(10, 6))
        self.canvas_disp = FigureCanvasTkAgg(self.fig_disp, self.disp_frame)
        self.canvas_disp.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Velocity plot
        self.fig_vel, self.ax_vel = plt.subplots(figsize=(10, 6))
        self.canvas_vel = FigureCanvasTkAgg(self.fig_vel, self.vel_frame)
        self.canvas_vel.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Acceleration plot
        self.fig_acc, self.ax_acc = plt.subplots(figsize=(10, 6))
        self.canvas_acc = FigureCanvasTkAgg(self.fig_acc, self.acc_frame)
        self.canvas_acc.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Cam profile plot
        self.fig_profile, self.ax_profile = plt.subplots(figsize=(10, 10))
        self.canvas_profile = FigureCanvasTkAgg(self.fig_profile, self.profile_frame)
        self.canvas_profile.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def update_parameters(self):
        """Read parameters from GUI"""
        try:
            self.params.stroke = float(self.stroke_entry.get())
            self.params.base_circle_radius = float(self.base_radius_entry.get())
            self.params.rise_angle = float(self.rise_angle_entry.get())
            self.params.dwell1_angle = float(self.dwell1_entry.get())
            self.params.return_angle = float(self.return_angle_entry.get())
            self.params.dwell2_angle = float(self.dwell2_entry.get())
            self.params.motion_law_rise = self.rise_law_combo.get()
            self.params.motion_law_return = self.return_law_combo.get()
            self.params.resolution = int(self.resolution_entry.get())
            return True
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input: {e}")
            return False
    
    def generate_profile(self):
        """Generate and plot cam profile"""
        if not self.update_parameters():
            return
        
            # -------- ANGLE VERIFICATION --------
        total_angle = (
            self.params.rise_angle +
            self.params.dwell1_angle +
            self.params.return_angle +
            self.params.dwell2_angle)
    

        if abs(total_angle - 360.0) > 1e-6:
            messagebox.showerror(
                "Angle Error",
                f"Total cam angle must be 360°.\nCurrent total = {total_angle:.2f}°")
            return
        # Create geometry calculator
        self.cam_geometry = CamGeometry(self.params)
        
        # Calculate profiles
        self.cam_geometry.calculate_displacement_diagram()
        self.cam_geometry.calculate_pitch_curve()
        
        # Update plots
        self.plot_displacement()
        self.plot_velocity()
        self.plot_acceleration()
        self.plot_cam_profile()
        
        messagebox.showinfo("Success", "Cam profile generated successfully!")
    
    def plot_displacement(self):
        """Plot displacement diagram"""
        data = self.cam_geometry.get_displacement_data()
        
        self.ax_disp.clear()
        self.ax_disp.plot(data['angles_deg'], data['displacement'], 'b-', linewidth=2)
        self.ax_disp.set_xlabel('Cam Angle (degrees)', fontsize=12)
        self.ax_disp.set_ylabel('Displacement (mm)', fontsize=12)
        self.ax_disp.set_title('Follower Displacement Diagram', fontsize=14, fontweight='bold')
        self.ax_disp.grid(True, alpha=0.3)
        self.ax_disp.set_xlim(0, 360)
        self.fig_disp.tight_layout()
        self.canvas_disp.draw()
    
    def plot_velocity(self):
        """Plot velocity diagram"""
        data = self.cam_geometry.get_displacement_data()
        
        self.ax_vel.clear()
        self.ax_vel.plot(data['angles_deg'], data['velocity'], 'g-', linewidth=2)
        self.ax_vel.set_xlabel('Cam Angle (degrees)', fontsize=12)
        self.ax_vel.set_ylabel('Velocity (mm/rad)', fontsize=12)
        self.ax_vel.set_title('Follower Velocity Diagram', fontsize=14, fontweight='bold')
        self.ax_vel.grid(True, alpha=0.3)
        self.ax_vel.set_xlim(0, 360)
        self.ax_vel.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.fig_vel.tight_layout()
        self.canvas_vel.draw()
    
    def plot_acceleration(self):
        """Plot acceleration diagram"""
        data = self.cam_geometry.get_displacement_data()
        
        self.ax_acc.clear()
        self.ax_acc.plot(data['angles_deg'], data['acceleration'], 'r-', linewidth=2)
        self.ax_acc.set_xlabel('Cam Angle (degrees)', fontsize=12)
        self.ax_acc.set_ylabel('Acceleration (mm/rad²)', fontsize=12)
        self.ax_acc.set_title('Follower Acceleration Diagram', fontsize=14, fontweight='bold')
        self.ax_acc.grid(True, alpha=0.3)
        self.ax_acc.set_xlim(0, 360)
        self.ax_acc.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.fig_acc.tight_layout()
        self.canvas_acc.draw()
    
    def plot_cam_profile(self):
        """Plot cam profile"""
        x, y = self.cam_geometry.get_profile_coordinates()
        
        self.ax_profile.clear()
        self.ax_profile.plot(x, y, 'b-', linewidth=2, label='Cam Profile')
        
        # Add base circle
        circle_base = plt.Circle((0, 0), self.params.base_circle_radius, 
                                fill=False, color='gray', linestyle='--', 
                                label='Base Circle')
        self.ax_profile.add_patch(circle_base)
        
        # Add prime circle
        circle_prime = plt.Circle((0, 0), 
                                 self.params.base_circle_radius + self.params.stroke,
                                 fill=False, color='green', linestyle='--',
                                 label='Prime Circle')
        self.ax_profile.add_patch(circle_prime)
        
        # Add center
        self.ax_profile.plot(0, 0, 'ko', markersize=8, label='Center')
        
        self.ax_profile.set_xlabel('X (mm)', fontsize=12)
        self.ax_profile.set_ylabel('Y (mm)', fontsize=12)
        self.ax_profile.set_title('Cam Profile (Knife-Edge Follower)', 
                                 fontsize=14, fontweight='bold')
        self.ax_profile.grid(True, alpha=0.3)
        self.ax_profile.axis('equal')
        self.ax_profile.legend()
        self.fig_profile.tight_layout()
        self.canvas_profile.draw()
    
    def export_csv(self):
        """Export cam profile to CSV"""
        if self.cam_geometry is None:
            messagebox.showwarning("Warning", "Please generate cam profile first!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filename:
            x, y = self.cam_geometry.get_profile_coordinates()
            disp_data = self.cam_geometry.get_displacement_data()
            
            if CamExporter.export_to_csv(filename, x, y, disp_data):
                messagebox.showinfo("Success", f"Exported to {filename}")
            else:
                messagebox.showerror("Error", "Failed to export CSV")
    
    def export_dxf(self):
        """Export cam profile to DXF"""
        if self.cam_geometry is None:
            messagebox.showwarning("Warning", "Please generate cam profile first!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".dxf",
            filetypes=[("DXF files", "*.dxf"), ("All files", "*.*")]
        )
        
        if filename:
            x, y = self.cam_geometry.get_profile_coordinates()
            
            if CamExporter.export_to_dxf(filename, x, y, self.params):
                messagebox.showinfo("Success", f"Exported to {filename}")
            else:
                messagebox.showerror("Error", "Failed to export DXF")
    
    def save_parameters(self):
        """Save parameters to JSON file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            if not self.update_parameters():
                return
            
            if CamExporter.export_parameters(filename, self.params):
                messagebox.showinfo("Success", f"Parameters saved to {filename}")
            else:
                messagebox.showerror("Error", "Failed to save parameters")


# ==================== MAIN APPLICATION ====================
def main():
    """Main application entry point"""
    root = tk.Tk()
    app = CamGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()


# ==================== EXAMPLE USAGE (Command Line) ====================
def generate_example_cams():
    """
    Generate example cam profiles for different motion laws
    This function can be run independently to create sample files
    """
    examples = [
        {
            'name': 'SHM_Rise_Return',
            'params': CamParameters(
                stroke=30,
                base_circle_radius=50,
                rise_angle=120,
                dwell1_angle=30,
                return_angle=120,
                dwell2_angle=30,
                motion_law_rise='SHM',
                motion_law_return='SHM',
                resolution=360
            )
        },
        {
            'name': 'Cycloidal_High_Speed',
            'params': CamParameters(
                stroke=25,
                base_circle_radius=60,
                rise_angle=90,
                dwell1_angle=45,
                return_angle=90,
                dwell2_angle=135,
                motion_law_rise='Cycloidal',
                motion_law_return='Cycloidal',
                resolution=720
            )
        },
        {
            'name': 'Mixed_Motion_Laws',
            'params': CamParameters(
                stroke=40,
                base_circle_radius=45,
                rise_angle=150,
                dwell1_angle=0,
                return_angle=150,
                dwell2_angle=60,
                motion_law_rise='Uniform Acceleration',
                motion_law_return='Cycloidal',
                resolution=360
            )
        }
    ]
    
    print("Generating example cam profiles...")
    
    for example in examples:
        print(f"\nGenerating: {example['name']}")
        
        # Create geometry
        geom = CamGeometry(example['params'])
        geom.calculate_displacement_diagram()
        geom.calculate_pitch_curve()
        
        # Get coordinates
        x, y = geom.get_profile_coordinates()
        disp_data = geom.get_displacement_data()
        
        # Export CSV
        csv_filename = f"example_{example['name']}.csv"
        CamExporter.export_to_csv(csv_filename, x, y, disp_data)
        print(f"  - CSV exported: {csv_filename}")
        
        # Export DXF
        if DXF_AVAILABLE:
            dxf_filename = f"example_{example['name']}.dxf"
            CamExporter.export_to_dxf(dxf_filename, x, y, example['params'])
            print(f"  - DXF exported: {dxf_filename}")
        
        # Export parameters
        json_filename = f"example_{example['name']}_params.json"
        CamExporter.export_parameters(json_filename, example['params'])
        print(f"  - Parameters saved: {json_filename}")
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f"Cam Profile: {example['name']}", fontsize=16, fontweight='bold')
        
        # Displacement
        axes[0, 0].plot(disp_data['angles_deg'], disp_data['displacement'], 'b-', linewidth=2)
        axes[0, 0].set_xlabel('Cam Angle (deg)')
        axes[0, 0].set_ylabel('Displacement (mm)')
        axes[0, 0].set_title('Displacement')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Velocity
        axes[0, 1].plot(disp_data['angles_deg'], disp_data['velocity'], 'g-', linewidth=2)
        axes[0, 1].set_xlabel('Cam Angle (deg)')
        axes[0, 1].set_ylabel('Velocity (mm/rad)')
        axes[0, 1].set_title('Velocity')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Acceleration
        axes[1, 0].plot(disp_data['angles_deg'], disp_data['acceleration'], 'r-', linewidth=2)
        axes[1, 0].set_xlabel('Cam Angle (deg)')
        axes[1, 0].set_ylabel('Acceleration (mm/rad²)')
        axes[1, 0].set_title('Acceleration')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Cam Profile
        axes[1, 1].plot(x, y, 'b-', linewidth=2, label='Cam Profile')
        circle_base = plt.Circle((0, 0), example['params'].base_circle_radius,
                                fill=False, color='gray', linestyle='--', label='Base Circle')
        axes[1, 1].add_patch(circle_base)
        axes[1, 1].plot(0, 0, 'ko', markersize=8)
        axes[1, 1].set_xlabel('X (mm)')
        axes[1, 1].set_ylabel('Y (mm)')
        axes[1, 1].set_title('Cam Profile')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].axis('equal')
        axes[1, 1].legend()
        
        plt.tight_layout()
        plot_filename = f"example_{example['name']}_plots.png"
        plt.savefig(plot_filename, dpi=150)
        print(f"  - Plots saved: {plot_filename}")
        plt.close()
    
    print("\n✓ All example cam profiles generated successfully!")
    print("\nTo run the GUI, execute: python cam_profile_generator.py")
    print("Or call generate_example_cams() to create more examples")


# Uncomment the line below to generate examples when running the script
#generate_example_cams()