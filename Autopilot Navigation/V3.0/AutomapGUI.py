import tkinter as tk
from tkinter import ttk, messagebox
import tkintermapview
import os
from PIL import Image, ImageTk
import requests
import serial
import serial.tools.list_ports
import threading
import time
from pymavlink import mavutil

class MapApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Drone Map Waypoint Collector")
        self.master.geometry("900x700")

        # Modern color palette
        self.bg_gradient_start = "#232526"  # dark gray
        self.bg_gradient_end = "#414345"    # lighter gray
        self.panel_color = "#2c3e50"
        self.button_color = "#2980b9"
        self.button_hover = "#3498db"
        self.button_fg = "#ffffff"
        self.status_connected = "#27ae60"
        self.status_disconnected = "#e74c3c"
        self.map_bg = "#222831"
        self.accent = "#f1c40f"

        self.waypoints = []
        self.path = None
        self.initial_waypoint = None
        self.mavlink_connection = None
        self.serial_connection = None
        self.is_connected = False
        self.current_lat = 41.672824
        self.current_lon = -71.441307
        self.telemetry_thread = None
        self.stop_telemetry = False

        # Gradient background using Canvas
        self.bg_canvas = tk.Canvas(self.master, width=900, height=700, highlightthickness=0)
        self.bg_canvas.place(x=0, y=0, relwidth=1, relheight=1)
        self.draw_gradient(self.bg_canvas, 900, 700, self.bg_gradient_start, self.bg_gradient_end)

        # Create a frame for the black box (now a modern dark panel)
        self.black_box = tk.Frame(self.master, bg=self.panel_color, width=112)
        self.black_box.pack(side="left", fill="y")

        # Create a frame for the map and controls
        self.right_frame = tk.Frame(self.master, highlightthickness=0)
        self.right_frame.pack(side="right", fill="both", expand=True)

        # Create control panel
        self.create_control_panel()

        # Create map widget
        self.map_widget = tkintermapview.TkinterMapView(self.right_frame, width=788, height=500, corner_radius=12)
        self.map_widget.pack(pady=10)

        # Battery percentage label and icon (top right corner of map area)
        self.battery_frame = tk.Frame(self.right_frame, highlightthickness=0)
        self.battery_frame.place(relx=1.0, y=0, anchor="ne", x=-10, rely=0.01)
        self.battery_icon = tk.Canvas(self.battery_frame, width=28, height=14, highlightthickness=0)
        self.battery_icon.pack(side="left", padx=(0,2))
        self.draw_battery_icon(self.battery_icon, 100)  # Start with 100%
        self.battery_label = tk.Label(self.battery_frame, text="--%", fg=self.accent, font=("Arial", 11, "bold"))
        self.battery_label.pack(side="left")

        # Set initial position and zoom
        self.map_widget.set_position(self.current_lat, self.current_lon)
        self.map_widget.set_zoom(12)

        # Load plane image
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.plane_image_orig = Image.open(os.path.join(self.current_path, "images", "plane.png")).resize((40, 40))
        self.plane_image = ImageTk.PhotoImage(self.plane_image_orig)
        self.plane_heading = 0  # Store last heading
        self.plane_image_cache = {}

        # Add initial waypoint with plane image
        self.add_waypoint((self.current_lat, self.current_lon), is_initial=True)

        # Bind click event to map
        self.map_widget.add_left_click_map_command(self.add_waypoint)

        # Create submit button (larger and green)
        self.submit_button = tk.Button(self.right_frame, text="Submit Waypoints", command=self.submit_waypoints,
                                       bg="green", fg="white", font=("Arial", 14), padx=20, pady=10)
        self.submit_button.pack(pady=10)

    def draw_gradient(self, canvas, width, height, color1, color2):
        # Simulate a vertical gradient by drawing many rectangles
        import colorsys
        r1, g1, b1 = self.master.winfo_rgb(color1)
        r2, g2, b2 = self.master.winfo_rgb(color2)
        r1, g1, b1 = r1//256, g1//256, b1//256
        r2, g2, b2 = r2//256, g2//256, b2//256
        steps = height
        for i in range(steps):
            ratio = i / steps
            r = int(r1 + (r2 - r1) * ratio)
            g = int(g1 + (g2 - g1) * ratio)
            b = int(b1 + (b2 - b1) * ratio)
            color = f'#{r:02x}{g:02x}{b:02x}'
            canvas.create_rectangle(0, i, width, i+1, outline=color, fill=color)

    def draw_battery_icon(self, canvas, percent):
        # Draw a simple battery icon with fill based on percent
        canvas.delete("all")
        # Battery body
        canvas.create_rectangle(2, 2, 24, 12, outline="#888", width=2, fill="#222")
        # Battery tip
        canvas.create_rectangle(24, 5, 27, 9, outline="#888", width=1, fill="#888")
        # Fill level
        fill_width = int(20 * max(0, min(percent, 100)) / 100)
        fill_color = self.accent if percent > 20 else "#e74c3c"
        canvas.create_rectangle(4, 4, 4+fill_width, 10, outline="", fill=fill_color)

    def create_control_panel(self):
        """Create the control panel with COM port and baud rate selection and connection controls"""
        control_frame = tk.Frame(self.right_frame, bg=self.panel_color, relief="raised", bd=0)
        control_frame.pack(fill="x", padx=10, pady=12)

        # COM Port Selection
        tk.Label(control_frame, text="COM Port:", bg=self.panel_color, fg=self.button_fg, font=("Arial", 10, "bold")).pack(side="left", padx=5)
        
        self.com_port_var = tk.StringVar()
        self.com_port_combo = ttk.Combobox(control_frame, textvariable=self.com_port_var, width=10)
        self.com_port_combo.pack(side="left", padx=5)
        self.refresh_com_ports()

        # Baud Rate Selection
        tk.Label(control_frame, text="Baud Rate:", bg=self.panel_color, fg=self.button_fg, font=("Arial", 10, "bold")).pack(side="left", padx=5)
        self.baud_rate_var = tk.StringVar(value="115200")
        self.baud_rate_combo = ttk.Combobox(control_frame, textvariable=self.baud_rate_var, width=8, state="readonly")
        self.baud_rate_combo['values'] = ("57600", "115200", "38400")
        self.baud_rate_combo.pack(side="left", padx=5)
        self.baud_rate_combo.set("115200")

        # Refresh button
        self.refresh_btn = tk.Button(control_frame, text="Refresh", command=self.refresh_com_ports, 
                 bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=10, pady=4, activebackground=self.button_hover, activeforeground=self.button_fg)
        self.refresh_btn.pack(side="left", padx=5)
        self.refresh_btn.bind("<Enter>", lambda e: self.refresh_btn.config(bg=self.button_hover))
        self.refresh_btn.bind("<Leave>", lambda e: self.refresh_btn.config(bg=self.button_color))

        # Connect/Disconnect button
        self.connect_button = tk.Button(control_frame, text="Connect", command=self.toggle_connection,
                                       bg=self.status_connected, fg=self.button_fg, font=("Arial", 10, "bold"), relief="flat", bd=0, padx=16, pady=6, activebackground=self.button_hover, activeforeground=self.button_fg)
        self.connect_button.pack(side="left", padx=10)
        self.connect_button.bind("<Enter>", lambda e: self.connect_button.config(bg=self.button_hover))
        self.connect_button.bind("<Leave>", lambda e: self.connect_button.config(bg=self.status_connected if not self.is_connected else self.status_disconnected))

        # Status label
        self.status_label = tk.Label(control_frame, text="Disconnected", bg=self.panel_color, fg=self.status_disconnected, font=("Arial", 10))
        self.status_label.pack(side="right", padx=10)

        # Current position display
        pos_frame = tk.Frame(self.right_frame, bg="#232526", relief="sunken", bd=0)
        pos_frame.pack(fill="x", padx=10, pady=2)
        
        tk.Label(pos_frame, text="Current Position:", bg="#232526", fg=self.button_fg, font=("Arial", 9, "bold")).pack(side="left", padx=5)
        self.position_label = tk.Label(pos_frame, text="Lat: 0.0, Lon: 0.0", bg="#232526", fg=self.accent, font=("Arial", 9))
        self.position_label.pack(side="left", padx=5)

    def refresh_com_ports(self):
        """Refresh the list of available COM ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_combo['values'] = ports
        if ports and not self.com_port_var.get():
            self.com_port_var.set(ports[0])

    def toggle_connection(self):
        """Toggle MAVLink connection"""
        if not self.is_connected:
            self.connect_to_mavlink()
        else:
            self.disconnect_from_mavlink()

    def connect_to_mavlink(self):
        """Connect to MAVLink device"""
        com_port = self.com_port_var.get()
        baud_rate = int(self.baud_rate_var.get()) if self.baud_rate_var.get().isdigit() else 115200
        if not com_port:
            tk.messagebox.showerror("Error", "Please select a COM port")
            return

        try:
            print(f"Trying connection: {com_port} at {baud_rate} baud")
            self.mavlink_connection = mavutil.mavlink_connection(com_port, baud=baud_rate)
            print("Waiting for heartbeat...")
            self.mavlink_connection.wait_heartbeat(timeout=5)
            print(f"Successfully connected to {com_port} at {baud_rate} baud")

            self.is_connected = True
            self.connect_button.config(text="Disconnect", bg=self.status_connected)
            self.status_label.config(text="Connected", fg=self.status_connected)

            # Start telemetry thread
            self.stop_telemetry = False
            self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
            self.telemetry_thread.start()

            print(f"Connected to MAVLink device on {com_port}")

        except Exception as e:
            tk.messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            print(f"Connection error: {e}")

    def disconnect_from_mavlink(self):
        """Disconnect from MAVLink device"""
        self.stop_telemetry = True
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=1)
        
        if self.mavlink_connection:
            self.mavlink_connection.close()
            self.mavlink_connection = None
        
        self.is_connected = False
        self.connect_button.config(text="Connect", bg=self.status_disconnected)
        self.status_label.config(text="Disconnected", fg=self.status_disconnected)
        print("Disconnected from MAVLink device")

    def telemetry_loop(self):
        """Main telemetry loop to read GPS coordinates, heading, and battery percentage"""
        while not self.stop_telemetry and self.mavlink_connection:
            try:
                # Get GLOBAL_POSITION_INT message for heading and position
                msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    # Heading in centidegrees (0-35999), 0 = North
                    heading = getattr(msg, 'hdg', None)
                    if heading is not None and heading != 65535:
                        heading_deg = heading / 100.0
                        self.plane_heading = heading_deg
                        self.master.after(0, self.update_plane_heading, heading_deg)
                    # Update current position
                    self.current_lat = lat
                    self.current_lon = lon
                    # Update GUI in main thread
                    self.master.after(0, self.update_position_display, lat, lon)
                    # Update initial waypoint if this is the first GPS fix
                    if len(self.waypoints) == 1 and self.waypoints[0] == (41.672824, -71.441307):
                        self.master.after(0, self.update_initial_waypoint, lat, lon)
                # Also check for SYS_STATUS message for battery
                sys_status = self.mavlink_connection.recv_match(type='SYS_STATUS', blocking=False)
                if sys_status and hasattr(sys_status, 'battery_remaining'):
                    battery = sys_status.battery_remaining
                    if battery != -1:
                        self.master.after(0, self.update_battery_display, battery)
            except Exception as e:
                print(f"Telemetry error: {e}")
                time.sleep(0.1)

    def update_position_display(self, lat, lon):
        """Update the position display in the GUI"""
        self.position_label.config(text=f"Lat: {lat:.6f}, Lon: {lon:.6f}")

    def update_initial_waypoint(self, lat, lon):
        """Update the initial waypoint with actual GPS coordinates"""
        # Remove old initial waypoint
        if self.initial_waypoint:
            self.map_widget.delete(self.initial_waypoint)
        # Clear waypoints and add new initial position
        self.waypoints = [(lat, lon)]
        self.add_waypoint((lat, lon), is_initial=True)
        # After adding, update the heading icon if available
        self.update_plane_heading(self.plane_heading)
        # Center map on new position
        self.map_widget.set_position(lat, lon)

    def add_waypoint(self, coords, is_initial=False):
        lat, lon = coords
        self.waypoints.append((lat, lon))
        marker_text = "Initial Waypoint" if is_initial else f"Waypoint {len(self.waypoints)}"
        if is_initial:
            marker = self.map_widget.set_marker(lat, lon, text=marker_text, icon=self.plane_image, command=self.marker_callback)
            self.initial_waypoint = marker  # Store the initial waypoint marker
        else:
            self.map_widget.set_marker(lat, lon, text=marker_text)
        self.update_path()

    def update_path(self):
        if len(self.waypoints) > 1:
            if self.path:
                self.map_widget.delete(self.path)
            self.path = self.map_widget.set_path(self.waypoints, color="red", width=3)

    def submit_waypoints(self):
        print("Stored Waypoints:")
        for i, coords in enumerate(self.waypoints, 1):
            print(f"Waypoint {i}: Latitude {coords[0]}, Longitude {coords[1]}")
        
        # Send waypoints to server
        self.send_waypoints_to_server()
        
        # Clear waypoints after submission
        self.clear_waypoints()

    def marker_callback(self, marker):
        print(marker.text)

    def clear_waypoints(self):
        # Remove all markers
        self.map_widget.delete_all_marker()

        # Clear the path
        if self.path:
            self.map_widget.delete(self.path)
            self.path = None

        # Reset waypoints list and add back the initial waypoint
        self.waypoints = [(self.current_lat, self.current_lon)]
        self.add_waypoint((self.current_lat, self.current_lon), is_initial=True)

    def send_waypoints_to_server(self):
        base_url = "http://pcr.bounceme.net/insert_temp.php"
        
        # Convert waypoints to string format with space between coordinate pairs
        coords_str = " ".join([f"{lat},{lon}" for lat, lon in self.waypoints])
        
        params = {
            "temperature": 605.5,  # You might want to replace this with actual temperature data
            "id": 1,  # You might want to generate a unique ID
            "coords": coords_str
        }

        try:
            response = requests.get(base_url, params=params)
            if response.status_code == 200:
                print("Waypoints sent successfully")
                print(response.text)
            else:
                print(f"Failed to send waypoints. Status code: {response.status_code}")
        except requests.RequestException as e:
            print(f"An error occurred while sending waypoints: {e}")

    def update_battery_display(self, battery_percent):
        """Update the battery percentage display in the GUI and redraw the icon"""
        self.battery_label.config(text=f"{battery_percent}%")
        self.draw_battery_icon(self.battery_icon, battery_percent)

    def update_plane_heading(self, heading_deg):
        """Rotate the plane image and update the initial waypoint marker icon"""
        # Cache rotated images for performance
        heading_int = int(heading_deg)
        if heading_int in self.plane_image_cache:
            rotated_imgtk = self.plane_image_cache[heading_int]
        else:
            rotated_img = self.plane_image_orig.rotate(-heading_deg, resample=Image.BICUBIC, expand=True)
            rotated_imgtk = ImageTk.PhotoImage(rotated_img)
            self.plane_image_cache[heading_int] = rotated_imgtk
        # Update the marker icon by deleting and recreating the marker
        if self.initial_waypoint and self.waypoints:
            lat, lon = self.waypoints[0]
            self.map_widget.delete(self.initial_waypoint)
            self.initial_waypoint = self.map_widget.set_marker(lat, lon, text="Drone", icon=rotated_imgtk, command=self.marker_callback)

if __name__ == "__main__":
    root = tk.Tk()
    app = MapApp(root)
    root.mainloop()
