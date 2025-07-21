import tkinter as tk
from tkinter import ttk, messagebox
import tkintermapview
import os
from PIL import Image, ImageTk, ImageDraw
import requests
import serial
import serial.tools.list_ports
import threading
import time
from pymavlink import mavutil
import math

class MapApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Drone Map Waypoint Collector")
        self.master.geometry("1200x800")  # Increased width for telemetry

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
        self.current_alt = 0.0  # Track current altitude in meters
        self.plane_heading = 0  # Store last heading
        self.telemetry_thread = None
        self.stop_telemetry = False

        # Gradient background using Canvas
        self.bg_canvas = tk.Canvas(self.master, width=900, height=700, highlightthickness=0)
        self.bg_canvas.place(x=0, y=0, relwidth=1, relheight=1)
        self.draw_gradient(self.bg_canvas, 900, 700, self.bg_gradient_start, self.bg_gradient_end)

        # Create a frame for the black box (now a modern dark panel)
        self.black_box = tk.Frame(self.master, bg=self.panel_color, width=112)
        self.black_box.pack(side="left", fill="y")

        # --- Directional and Arming Controls in Sidebar ---
        self.sidebar_controls = tk.Frame(self.black_box, bg=self.panel_color)
        self.sidebar_controls.pack(pady=20, padx=8, anchor="n")

        # Move Forward
        tk.Label(self.sidebar_controls, text="Forward Dist (m):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(0,2))
        self.forward_distance_var = tk.StringVar(value="20.0")
        self.forward_distance_entry = ttk.Entry(self.sidebar_controls, textvariable=self.forward_distance_var, width=8)
        self.forward_distance_entry.pack(pady=2)
        self.move_forward_btn = tk.Button(self.sidebar_controls, text="Move Forward", command=self.move_forward_from_entry, bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2)
        self.move_forward_btn.pack(pady=4)

        # Increase Altitude
        tk.Label(self.sidebar_controls, text="Alt + (m):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(8,2))
        self.alt_increase_var = tk.StringVar(value="2.0")
        self.alt_increase_entry = ttk.Entry(self.sidebar_controls, textvariable=self.alt_increase_var, width=8)
        self.alt_increase_entry.pack(pady=2)
        self.increase_alt_btn = tk.Button(self.sidebar_controls, text="Increase Alt", command=self.increase_altitude_from_entry, bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2)
        self.increase_alt_btn.pack(pady=4)

        # Decrease Altitude
        tk.Label(self.sidebar_controls, text="Alt - (m):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(8,2))
        self.alt_decrease_var = tk.StringVar(value="2.0")
        self.alt_decrease_entry = ttk.Entry(self.sidebar_controls, textvariable=self.alt_decrease_var, width=8)
        self.alt_decrease_entry.pack(pady=2)
        self.decrease_alt_btn = tk.Button(self.sidebar_controls, text="Decrease Alt", command=self.decrease_altitude_from_entry, bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2)
        self.decrease_alt_btn.pack(pady=4)

        # Rotate In Place
        tk.Label(self.sidebar_controls, text="Rotate (deg):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(8,2))
        self.yaw_angle_var = tk.StringVar(value="45.0")
        self.yaw_angle_entry = ttk.Entry(self.sidebar_controls, textvariable=self.yaw_angle_var, width=8)
        self.yaw_angle_entry.pack(pady=2)
        self.rotate_btn = tk.Button(self.sidebar_controls, text="Rotate", command=self.rotate_in_place_from_entry, bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2)
        self.rotate_btn.pack(pady=4)

        # Translate
        tk.Label(self.sidebar_controls, text="Trans Angle (deg):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(8,2))
        self.translate_angle_var = tk.StringVar(value="0.0")
        self.translate_angle_entry = ttk.Entry(self.sidebar_controls, textvariable=self.translate_angle_var, width=8)
        self.translate_angle_entry.pack(pady=2)
        tk.Label(self.sidebar_controls, text="Trans Dist (m):", bg=self.panel_color, fg=self.button_fg, font=("Arial", 9)).pack(pady=(2,2))
        self.translate_distance_var = tk.StringVar(value="10.0")
        self.translate_distance_entry = ttk.Entry(self.sidebar_controls, textvariable=self.translate_distance_var, width=8)
        self.translate_distance_entry.pack(pady=2)
        self.translate_btn = tk.Button(self.sidebar_controls, text="Translate", command=self.translate_from_entry, bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2)
        self.translate_btn.pack(pady=4)

        # Manual Mode and Arm
        self.manual_arm_var = tk.BooleanVar(value=False)
        self.manual_arm_checkbox = ttk.Checkbutton(
            self.sidebar_controls,
            text="Set MANUAL mode and ARM",
            variable=self.manual_arm_var,
            command=self.set_manual_and_arm
        )
        self.manual_arm_checkbox.pack(pady=(16,4))

        # --- Mission Query Button ---
        self.query_mission_btn = tk.Button(
            self.sidebar_controls,
            text="Query Mission",
            command=self.query_mission_waypoints,
            bg=self.button_color, fg=self.button_fg, font=("Arial", 8), relief="flat", bd=0, padx=6, pady=2
        )
        self.query_mission_btn.pack(pady=8)

        # Create a frame for the map and controls
        self.right_frame = tk.Frame(self.master, highlightthickness=0)
        self.right_frame.pack(side="right", fill="both", expand=True)

        # --- Telemetry Display Frame ---
        self.telemetry_frame = tk.Frame(self.right_frame, bg="#181818")
        self.telemetry_frame.pack(side="right", fill="y", padx=10, pady=10, anchor="n")

        # Create control panel
        self.create_control_panel()

        # --- Map and Button Container ---
        map_button_container = tk.Frame(self.right_frame)
        map_button_container.pack(side="left", anchor="n", pady=10)

        # Create map widget
        self.map_widget = tkintermapview.TkinterMapView(map_button_container, width=788, height=500, corner_radius=12)
        self.map_widget.grid(row=0, column=0, pady=(0, 20))

        # Centered button frame below the map
        button_frame = tk.Frame(map_button_container)
        button_frame.grid(row=1, column=0)
        self.submit_button = tk.Button(button_frame, text="Submit Waypoints", command=self.submit_waypoints,
                                       bg="green", fg="white", font=("Arial", 14), padx=20, pady=10)
        self.submit_button.pack(side="left", padx=(0, 20))
        self.start_mission_button = tk.Button(button_frame, text="Start Mission", command=self.start_mission,
                                             bg="#2980b9", fg="white", font=("Arial", 14), padx=20, pady=10)
        self.start_mission_button.pack(side="left")

        # Battery percentage label and icon (top right corner of map area)
        self.battery_frame = tk.Frame(self.right_frame, highlightthickness=0)
        self.battery_frame.place(relx=0.8, y=0, anchor="ne", x=-10, rely=0.01)
        self.battery_icon = tk.Canvas(self.battery_frame, width=28, height=14, highlightthickness=0)
        self.battery_icon.pack(side="left", padx=(0,2))
        self.draw_battery_icon(self.battery_icon, 100)  # Start with 100%
        self.battery_label = tk.Label(self.battery_frame, text="--%", fg=self.accent, font=("Arial", 11, "bold"))
        self.battery_label.pack(side="left")

        # --- Telemetry Instruments ---
        self.init_telemetry_instruments()

        # Set initial position and zoom
        self.map_widget.set_position(self.current_lat, self.current_lon)
        self.map_widget.set_zoom(12)

        # Load plane image
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.plane_image_orig = Image.open(os.path.join(self.current_path, "images", "plane.png")).resize((40, 40))
        self.plane_image = ImageTk.PhotoImage(self.plane_image_orig)
        self.plane_heading = 0  # Store last heading
        self.plane_image_cache = {}

        # Add live drone marker (plane icon) -- will be updated with telemetry
        self.drone_marker = None
        self.update_drone_marker(self.current_lat, self.current_lon, self.plane_heading)

        # Bind click event to map
        self.map_widget.add_left_click_map_command(self.add_waypoint)

        # Run test ramp for telemetry on startup
        self.master.after(1000, self.test_ramp)

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

            # Center and zoom the map on the drone's current location 3 seconds after connect
            self.master.after(3000, self.zoom_to_drone_location)

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
        """Main telemetry loop to read GPS coordinates, heading, battery, and attitude (roll/pitch), and update speed/altitude indicators"""
        while not self.stop_telemetry and self.mavlink_connection:
            try:
                # Get GLOBAL_POSITION_INT message for heading, position, speed, and altitude
                msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0  # Altitude in meters
                    self.current_alt = alt
                    # Heading in centidegrees (0-35999), 0 = North
                    heading = getattr(msg, 'hdg', None)
                    if heading is not None and heading != 65535:
                        heading_deg = heading / 100.0
                        self.plane_heading = heading_deg
                        self.master.after(0, self.update_plane_heading, heading_deg)
                    # Update current position
                    self.current_lat = lat
                    self.current_lon = lon
                    # Update drone marker on the map
                    self.master.after(0, self.update_drone_marker, lat, lon, self.plane_heading)
                    # Update GUI in main thread
                    self.master.after(0, self.update_position_display, lat, lon)
                    # --- Update telemetry altitude (convert meters to feet) ---
                    self.telemetry_altitude = alt * 3.28084
                    # --- Update telemetry speed (ground speed in knots) ---
                    vx = getattr(msg, 'vx', 0)
                    vy = getattr(msg, 'vy', 0)
                    vz = getattr(msg, 'vz', 0)
                    # vx, vy, vz are in cm/s; ground speed is sqrt(vx^2 + vy^2)
                    ground_speed_cms = math.sqrt(vx**2 + vy**2)
                    ground_speed_ms = ground_speed_cms / 100.0
                    ground_speed_knots = ground_speed_ms * 1.94384
                    self.telemetry_speed = ground_speed_knots
                # Also check for SYS_STATUS message for battery
                sys_status = self.mavlink_connection.recv_match(type='SYS_STATUS', blocking=False)
                if sys_status and hasattr(sys_status, 'battery_remaining'):
                    battery = sys_status.battery_remaining
                    if battery != -1:
                        self.master.after(0, self.update_battery_display, battery)
                # --- Listen for ATTITUDE message for roll/pitch ---
                att_msg = self.mavlink_connection.recv_match(type='ATTITUDE', blocking=False)
                if att_msg:
                    # roll, pitch in radians; convert to degrees
                    roll_deg = math.degrees(att_msg.roll)
                    pitch_deg = math.degrees(att_msg.pitch)
                    self.telemetry_roll = roll_deg
                    self.telemetry_pitch = pitch_deg
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

    def add_waypoint(self, coords):
        lat, lon = coords
        # If this is the first waypoint, use the drone's current location
        if not self.waypoints:
            lat = self.current_lat
            lon = self.current_lon
        self.waypoints.append((lat, lon))
        marker_text = f"Waypoint {len(self.waypoints)}"
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
        # Send waypoints to Pixhawk via MAVLink
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        # Run mission upload in a background thread
        threading.Thread(target=self._threaded_send_waypoints_to_pixhawk, daemon=True).start()

    def _threaded_send_waypoints_to_pixhawk(self):
        try:
            self.send_waypoints_to_pixhawk()
            self.master.after(0, lambda: tk.messagebox.showinfo("Waypoints Sent", "Waypoints sent to Pixhawk via MAVLink."))
            self.master.after(0, self.send_waypoints_to_server)
            self.master.after(0, self.clear_waypoints)
        except Exception as e:
            err_msg = str(e)
            self.master.after(0, lambda: tk.messagebox.showerror("Error", f"Failed to send waypoints to Pixhawk: {err_msg}"))

    def send_waypoints_to_pixhawk(self):
        # Clear all existing missions
        self.mavlink_connection.mav.mission_clear_all_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component
        )
        time.sleep(0.5)
        # Send mission count
        n_wps = len(self.waypoints)
        self.mavlink_connection.mav.mission_count_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            n_wps
        )
        time.sleep(0.2)
        # Respond to MISSION_REQUEST or MISSION_REQUEST_INT for each waypoint
        for i, (lat, lon) in enumerate(self.waypoints):
            req_ok = False
            for _ in range(50):  # up to 5 seconds
                msg = self.mavlink_connection.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=False)
                if msg and msg.seq == i:
                    req_ok = True
                    break
                time.sleep(0.1)
            if not req_ok:
                raise RuntimeError(f"Timeout waiting for MISSION_REQUEST or MISSION_REQUEST_INT for seq {i}")
            # Always respond with MISSION_ITEM_INT (modern, preferred)
            self.mavlink_connection.mav.mission_item_int_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                i,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2 if i == n_wps-1 else 0,  # current (2=auto continue, 0=not current)
                1,  # autocontinue
                0, 0, 0, 0,  # param1-4: hold, acceptance radius, pass radius, yaw
                int(lat * 1e7),
                int(lon * 1e7),
                20 if hasattr(self, 'current_alt') and self.current_alt else 20,  # Altitude (default 20m)
                0  # mission_type
            )
            time.sleep(0.05)
        # Wait for MISSION_ACK
        ack_ok = False
        for _ in range(50):  # up to 5 seconds
            msg = self.mavlink_connection.recv_match(type='MISSION_ACK', blocking=False)
            if msg:
                ack_ok = True
                break
            time.sleep(0.1)
        if not ack_ok:
            raise RuntimeError("Timeout waiting for MISSION_ACK after sending waypoints")

    def start_mission(self):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        # PX4: AUTO.MISSION = 4, ArduPilot: AUTO = 4
        PX4_CUSTOM_MAIN_MODE_AUTO = 4
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 5
        try:
            # 1. Send COMMAND_LONG (MAV_CMD_DO_SET_MODE)
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: base mode
                PX4_CUSTOM_MAIN_MODE_AUTO,          # param2: custom mode (AUTO)
                0, 0, 0, 0, 0
            )
            time.sleep(0.2)
            # 2. Send SET_MODE (for ArduPilot compatibility)
            self.mavlink_connection.mav.set_mode_send(
                self.mavlink_connection.target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_CUSTOM_MAIN_MODE_AUTO
            )
            tk.messagebox.showinfo("Mission Started", "Sent commands to enter AUTO/MISSION mode.")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to start mission: {e}")

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
        """Rotate the plane image and update the drone marker icon at the current GPS location"""
        self.update_drone_marker(self.current_lat, self.current_lon, heading_deg)

    def update_drone_marker(self, lat, lon, heading_deg):
        # Cache rotated images for performance
        heading_int = int(heading_deg)
        if heading_int in self.plane_image_cache:
            rotated_imgtk = self.plane_image_cache[heading_int]
        else:
            rotated_img = self.plane_image_orig.rotate(-heading_deg, resample=Image.BICUBIC, expand=True)
            rotated_imgtk = ImageTk.PhotoImage(rotated_img)
            self.plane_image_cache[heading_int] = rotated_imgtk

        # Only update or recreate the drone marker
        if self.drone_marker is not None:
            try:
                # Try to update position and icon if supported
                if hasattr(self.drone_marker, 'set_position'):
                    self.drone_marker.set_position(lat, lon)
                else:
                    # If not supported, recreate
                    raise AttributeError
                if hasattr(self.drone_marker, 'set_icon'):
                    self.drone_marker.set_icon(rotated_imgtk)
            except Exception:
                # If not supported, delete and recreate
                try:
                    self.map_widget.delete(self.drone_marker)
                except Exception:
                    pass
                self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone", icon=rotated_imgtk)
        else:
            self.drone_marker = self.map_widget.set_marker(lat, lon, text="Drone", icon=rotated_imgtk)

    # --- Directional Command Methods ---
    def move_forward_from_entry(self):
        try:
            distance = float(self.forward_distance_var.get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter a valid number for forward distance.")
            return
        self.move_forward(distance)

    def move_forward(self, distance):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        lat = self.current_lat
        lon = self.current_lon
        alt = self.current_alt
        heading_deg = self.plane_heading
        heading_rad = math.radians(heading_deg)
        earth_radius = 6378137.0
        dlat = (distance * math.cos(heading_rad)) / earth_radius
        dlon = (distance * math.sin(heading_rad)) / (earth_radius * math.cos(math.radians(lat)))
        new_lat = lat + math.degrees(dlat)
        new_lon = lon + math.degrees(dlon)
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0,
                heading_rad,
                new_lat, new_lon, alt
            )
            tk.messagebox.showinfo("Command Sent", f"Move to (DO_REPOSITION):\nLat: {new_lat:.7f}\nLon: {new_lon:.7f}\nAlt: {alt:.2f}")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to send move forward command: {e}")

    def increase_altitude_from_entry(self):
        try:
            increment = float(self.alt_increase_var.get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter a valid number for altitude increase.")
            return
        self.increase_altitude(increment)

    def increase_altitude(self, increment):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        lat = self.current_lat
        lon = self.current_lon
        alt = self.current_alt
        new_alt = alt + increment
        heading_deg = self.plane_heading
        heading_rad = math.radians(heading_deg)
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0,
                heading_rad,
                lat, lon, new_alt
            )
            tk.messagebox.showinfo("Command Sent", f"Increase Altitude:\nLat: {lat:.7f}\nLon: {lon:.7f}\nAlt: {new_alt:.2f}")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to send increase altitude command: {e}")

    def decrease_altitude_from_entry(self):
        try:
            decrement = float(self.alt_decrease_var.get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter a valid number for altitude decrease.")
            return
        self.decrease_altitude(decrement)

    def decrease_altitude(self, decrement):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        lat = self.current_lat
        lon = self.current_lon
        alt = self.current_alt
        new_alt = alt - decrement
        heading_deg = self.plane_heading
        heading_rad = math.radians(heading_deg)
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0,
                heading_rad,
                lat, lon, new_alt
            )
            tk.messagebox.showinfo("Command Sent", f"Decrease Altitude:\nLat: {lat:.7f}\nLon: {lon:.7f}\nAlt: {new_alt:.2f}")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to send decrease altitude command: {e}")

    def rotate_in_place_from_entry(self):
        try:
            yaw_angle = float(self.yaw_angle_var.get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter a valid number for yaw angle.")
            return
        direction = tk.messagebox.askquestion("Rotation Direction", "Rotate right (clockwise)? Click 'No' for left (counterclockwise).", icon='question')
        if direction == 'yes':
            self.rotate_in_place(yaw_angle)
        else:
            self.rotate_in_place(-yaw_angle)

    def rotate_in_place(self, yaw_angle):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        lat = self.current_lat
        lon = self.current_lon
        alt = self.current_alt
        # Update heading
        new_heading = (self.plane_heading + yaw_angle) % 360
        target_heading_rad = math.radians(new_heading)
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0,
                target_heading_rad,
                lat, lon, alt
            )
            self.plane_heading = new_heading  # Update local heading
            tk.messagebox.showinfo("Command Sent", f"Rotating in place to heading {new_heading:.1f} degrees")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to send rotate command: {e}")

    def translate_from_entry(self):
        try:
            angle = float(self.translate_angle_var.get())
            distance = float(self.translate_distance_var.get())
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter valid numbers for angle and distance.")
            return
        self.translate(angle, distance)

    def translate(self, angle, distance):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        lat = self.current_lat
        lon = self.current_lon
        alt = self.current_alt
        heading_deg = self.plane_heading
        move_direction_deg = (heading_deg + angle) % 360
        move_direction_rad = math.radians(move_direction_deg)
        earth_radius = 6378137.0
        dlat = (distance * math.cos(move_direction_rad)) / earth_radius
        dlon = (distance * math.sin(move_direction_rad)) / (earth_radius * math.cos(math.radians(lat)))
        new_lat = lat + math.degrees(dlat)
        new_lon = lon + math.degrees(dlon)
        heading_rad = math.radians(heading_deg)
        try:
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, 0, 0,
                heading_rad,
                new_lat, new_lon, alt
            )
            tk.messagebox.showinfo("Command Sent", f"Translate {distance}m at {angle}° rel. to heading (abs: {move_direction_deg:.1f}°)\nLat: {new_lat:.7f}\nLon: {new_lon:.7f}\nAlt: {alt:.2f}")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to send translate command: {e}")

    def set_manual_and_arm(self):
        if self.manual_arm_var.get():
            PX4_CUSTOM_MAIN_MODE_MANUAL = 1
            PX4_CUSTOM_MAIN_MODE_TAKEOFF = 4
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 5
            try:
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,  # confirmation
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: base mode
                    PX4_CUSTOM_MAIN_MODE_MANUAL,        # param2: custom mode
                    0, 0, 0, 0, 0
                )
                time.sleep(1)
                # Arm the drone
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,  # confirmation
                    1,  # param1: 1 to arm
                    0, 0, 0, 0, 0, 0
                )
                time.sleep(5)  # 5 second delay
                # Switch to TAKEOFF mode (PX4 custom main mode 4)
                self.mavlink_connection.mav.command_long_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,  # confirmation
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: base mode
                    PX4_CUSTOM_MAIN_MODE_TAKEOFF,       # param2: custom mode
                    0, 0, 0, 0, 0
                )
                # Automatically increase altitude by 5 meters
                self.increase_altitude(5)
                tk.messagebox.showinfo("Command Sent", "Set to MANUAL mode, ARMED, switched to TAKEOFF mode, and increased altitude by 5 meters.")
            except Exception as e:
                tk.messagebox.showerror("Error", f"Failed to set MANUAL mode, arm, switch to TAKEOFF, and increase altitude: {e}")

    # --- Mission Query Function ---
    def query_mission_waypoints(self):
        if not self.mavlink_connection:
            tk.messagebox.showerror("Error", "Not connected to MAVLink device.")
            return
        try:
            # Request mission count
            self.mavlink_connection.mav.mission_request_list_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component
            )
            # Wait for mission count
            count = None
            for _ in range(50):  # up to 5 seconds
                msg = self.mavlink_connection.recv_match(type='MISSION_COUNT', blocking=True, timeout=0.1)
                if msg:
                    count = msg.count
                    break
            if count is None:
                print("Timeout waiting for MISSION_COUNT")
                tk.messagebox.showerror("Error", "Timeout waiting for MISSION_COUNT.")
                return
            print(f"Mission has {count} waypoints:")
            # Request and print each waypoint
            waypoints = []
            for seq in range(count):
                self.mavlink_connection.mav.mission_request_int_send(
                    self.mavlink_connection.target_system,
                    self.mavlink_connection.target_component,
                    seq
                )
                for _ in range(50):
                    msg = self.mavlink_connection.recv_match(type=['MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=0.1)
                    if msg and msg.seq == seq:
                        if hasattr(msg, 'x') and hasattr(msg, 'y'):
                            lat = msg.x / 1e7
                            lon = msg.y / 1e7
                        elif hasattr(msg, 'param5') and hasattr(msg, 'param6'):
                            lat = msg.param5
                            lon = msg.param6
                        else:
                            lat = lon = None
                        waypoints.append((lat, lon))
                        print(f"  Waypoint {seq}: Lat {lat}, Lon {lon}")
                        break
            print(f"Total waypoints received: {len(waypoints)}")
        except Exception as e:
            print(f"Error querying mission waypoints: {e}")
            tk.messagebox.showerror("Error", f"Failed to query mission: {e}")

    def init_telemetry_instruments(self):
        # Import PIL.ImageDraw here if not already
        s = 1.0
        # Gyroscope
        self.gyro_canvas = tk.Canvas(self.telemetry_frame, width=int(200*s), height=int(200*s), bg='black', highlightthickness=0)
        self.gyro_canvas.pack(pady=(0,10))
        self.gyro_img = ImageTk.PhotoImage(Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 255)))
        self.gyro_canvas_img = self.gyro_canvas.create_image(0, 0, anchor="nw", image=self.gyro_img)
        # Speedometer
        self.speed_canvas = tk.Canvas(self.telemetry_frame, width=int(200*s), height=int(200*s), bg='black', highlightthickness=0)
        self.speed_canvas.pack(pady=(0,10))
        self.speed_img = ImageTk.PhotoImage(Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 255)))
        self.speed_canvas_img = self.speed_canvas.create_image(0, 0, anchor="nw", image=self.speed_img)
        # Altitude
        self.altitude_canvas = tk.Canvas(self.telemetry_frame, width=int(100*s), height=int(200*s), bg='black', highlightthickness=0)
        self.altitude_canvas.pack()
        self.altitude_img = ImageTk.PhotoImage(Image.new("RGBA", (int(100*s), int(200*s)), (0, 0, 0, 255)))
        self.altitude_canvas_img = self.altitude_canvas.create_image(0, 0, anchor="nw", image=self.altitude_img)
        # Telemetry values
        self.telemetry_speed = 0
        self.telemetry_altitude = 0
        self.telemetry_pitch = 0
        self.telemetry_roll = 0
        self._telemetry_running = True
        self.master.after(16, self.update_telemetry_instruments)

    def update_telemetry_instruments(self):
        s = 1.0
        # Draw gyroscope
        img = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 255))
        draw = ImageDraw.Draw(img)
        center_x, center_y = int(100*s), int(100*s)
        radius = int(90*s)
        pitch_offset = self.telemetry_pitch * 2 * s
        sky_bottom = max(0, min(int(200*s), int(100*s + pitch_offset)))
        earth_top = sky_bottom
        draw.rectangle([0, 0, int(200*s), sky_bottom], fill="skyblue")
        draw.rectangle([0, earth_top, int(200*s), int(200*s)], fill="saddlebrown")
        img = img.rotate(-self.telemetry_roll, center=(center_x, center_y), resample=Image.BICUBIC, expand=False)
        mask = Image.new("L", (int(200*s), int(200*s)), 0)
        mask_draw = ImageDraw.Draw(mask)
        mask_draw.ellipse([center_x - radius, center_y - radius, center_x + radius, center_y + radius], fill=255)
        img.putalpha(mask)
        ladder_img = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 0))
        ladder_draw = ImageDraw.Draw(ladder_img)
        ladder_width = 30*s*0.5
        tick_width = 40*s*0.5
        number_offset = 10*s
        for pitch in range(-50, 55, 5):
            if pitch == 0:
                continue
            offset = pitch * 2 * s
            y = center_y + pitch_offset - offset
            if (y < center_y - radius) or (y > center_y + radius):
                continue
            ladder_draw.line([(center_x - ladder_width, y), (center_x + ladder_width, y)], fill="white", width=int(2*s))
            if pitch % 10 == 0:
                ladder_draw.line([(center_x - tick_width, y), (center_x + tick_width, y)], fill="white", width=int(2*s))
            ladder_draw.text((center_x - ladder_width - number_offset, y - 7*s), str(abs(pitch)), fill="white")
            ladder_draw.text((center_x + ladder_width + 2, y - 7*s), str(abs(pitch)), fill="white")
        ladder_img = ladder_img.rotate(-self.telemetry_roll, center=(center_x, center_y), resample=Image.BICUBIC, expand=False)
        ladder_img_masked = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 0))
        ladder_img_masked.paste(ladder_img, (0, 0), mask)
        roll_img = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 0))
        roll_draw = ImageDraw.Draw(roll_img)
        for angle, label in [(-60, "60"), (-50, ""), (-45, "45"), (-40, ""), (-30, "30"), (-20, ""), (-10, ""), (0, ""), (10, ""), (20, ""), (30, "30"), (40, ""), (45, "45"), (50, ""), (60, "60")]:
            rad = math.radians(angle)
            x1 = center_x + (radius - 5*s) * math.sin(rad)
            y1 = center_y - (radius - 5*s) * math.cos(rad)
            x2 = center_x + (radius - 15*s) * math.sin(rad)
            y2 = center_y - (radius - 15*s) * math.cos(rad)
            roll_draw.line([(x1, y1), (x2, y2)], fill="white", width=int(2*s))
            if label:
                lx = center_x + (radius - 25*s) * math.sin(rad)
                ly = center_y - (radius - 25*s) * math.cos(rad)
                roll_draw.text((lx, ly), label, fill="white")
        roll_img = roll_img.rotate(-self.telemetry_roll, center=(center_x, center_y), resample=Image.BICUBIC, expand=False)
        roll_img_masked = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 0))
        roll_img_masked.paste(roll_img, (0, 0), mask)
        final_img = Image.alpha_composite(img, ladder_img_masked)
        final_img = Image.alpha_composite(final_img, roll_img_masked)
        # --- Draw red arrow indicator (center arrowhead) ---
        arrow_img = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 0))
        arrow_draw = ImageDraw.Draw(arrow_img)
        base_width = 30 * s
        height = 12.5 * s
        cx, cy = center_x, center_y
        arrow_points = [
            (cx - base_width // 2, cy + height // 2),
            (cx + base_width // 2, cy + height // 2),
            (cx, cy - height)
        ]
        arrow_draw.polygon(arrow_points, fill="red")
        final_img = Image.alpha_composite(final_img, arrow_img)
        self.gyro_img.paste(final_img)
        self.gyro_canvas.itemconfig(self.gyro_canvas_img, image=self.gyro_img)
        # Draw static overlays
        c = self.gyro_canvas
        c.delete("static")
        c.create_polygon(center_x - 8*s, center_y - radius + 6*s, center_x + 8*s, center_y - radius + 6*s, center_x, center_y - radius - 8*s, fill="white", tags="static")
        c.create_line(center_x - 20*s, center_y + 40*s, center_x + 20*s, center_y + 40*s, fill="white", width=int(4*s), tags="static")
        c.create_line(center_x, center_y + 40*s, center_x, center_y + 30*s, fill="white", width=int(4*s), tags="static")
        c.create_oval(center_x - 5*s, center_y + radius - 15*s, center_x + 5*s, center_y + radius - 5*s, fill="white", tags="static")
        c.create_line(center_x, center_y - radius + 10*s, center_x, center_y + radius - 10*s, fill="yellow", width=int(2*s), tags="static")
        # Speedometer
        img = Image.new("RGBA", (int(200*s), int(200*s)), (0, 0, 0, 255))
        draw = ImageDraw.Draw(img)
        draw.ellipse([10*s, 10*s, 190*s, 190*s], outline="white", width=int(2*s))
        draw.text((60*s, 5*s), "Speed (knots)", fill="white")
        for i in range(0, 181, 20):
            angle = math.radians(180 - i)
            x_start = 100*s + 80*s * math.cos(angle)
            y_start = 100*s - 80*s * math.sin(angle)
            x_end = 100*s + 90*s * math.cos(angle)
            y_end = 100*s - 90*s * math.sin(angle)
            draw.line([(x_start, y_start), (x_end, y_end)], fill="white", width=int(2*s))
            tx = 100*s + 65*s * math.cos(angle)
            ty = 100*s - 65*s * math.sin(angle)
            draw.text((tx - 8*s, ty - 7*s), str(i), fill="white")
        speed = self.telemetry_speed
        angle = math.radians(180 - speed)
        x_needle = 100*s + 70*s * math.cos(angle)
        y_needle = 100*s - 70*s * math.sin(angle)
        draw.line([(100*s, 100*s), (x_needle, y_needle)], fill="red", width=int(3*s))
        self.speed_img.paste(img)
        self.speed_canvas.itemconfig(self.speed_canvas_img, image=self.speed_img)
        # Altitude
        img = Image.new("RGBA", (int(100*s), int(200*s)), (0, 0, 0, 255))
        draw = ImageDraw.Draw(img)
        draw.rectangle([30*s, 20*s, 70*s, 180*s], outline="white", width=int(2*s))
        draw.text((10*s, 0*s), "Altitude (ft)", fill="white")
        # Scale: 0 to 1000 ft, ticks every 200 ft
        for i in range(0, 1001, 200):
            y = 180*s - (i / 1000) * 160*s
            draw.line([(30*s, y), (70*s, y)], fill="white", width=int(2*s))
            draw.text((0*s, y - 7*s), str(i), fill="white")
        altitude = self.telemetry_altitude
        # Clamp altitude to [0, 1000] for display
        altitude = max(0, min(altitude, 1000))
        y_alt = 180*s - (altitude / 1000) * 160*s
        draw.polygon([(70*s, y_alt), (80*s, y_alt - 10*s), (80*s, y_alt + 10*s)], fill="red")
        self.altitude_img.paste(img)
        self.altitude_canvas.itemconfig(self.altitude_canvas_img, image=self.altitude_img)
        if self._telemetry_running:
            self.master.after(16, self.update_telemetry_instruments)

    def test_ramp(self):
        def ramp():
            max_speed = 180
            max_altitude = 16000
            max_pitch = 60
            max_roll = 90
            step = 2
            original_pitch = self.telemetry_pitch
            original_roll = self.telemetry_roll
            for i in range(0, 101, step):
                self.telemetry_speed = max_speed * i / 100
                self.telemetry_altitude = max_altitude * i / 100
                self.telemetry_pitch = original_pitch
                self.telemetry_roll = original_roll
                time.sleep(0.02)
            for i in range(100, -1, -step):
                self.telemetry_speed = max_speed * i / 100
                self.telemetry_altitude = max_altitude * i / 100
                self.telemetry_pitch = original_pitch
                self.telemetry_roll = original_roll
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.telemetry_roll = -max_roll * i / 100
                self.telemetry_pitch = 0
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.telemetry_roll = -max_roll + (2 * max_roll * i / 100)
                self.telemetry_pitch = 0
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.telemetry_roll = max_roll - (max_roll * i / 100)
                self.telemetry_pitch = 0
                time.sleep(0.02)
            self.telemetry_roll = 0
            for i in range(0, 101, step):
                self.telemetry_pitch = max_pitch * i / 100
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.telemetry_pitch = max_pitch - (2 * max_pitch * i / 100)
                time.sleep(0.02)
            for i in range(0, 101, step):
                self.telemetry_pitch = -max_pitch + (max_pitch * i / 100)
                time.sleep(0.02)
            self.telemetry_pitch = 0
            self.telemetry_roll = 0
        threading.Thread(target=ramp, daemon=True).start()

    def zoom_to_drone_location(self):
        self.map_widget.set_position(self.current_lat, self.current_lon)
        self.map_widget.set_zoom(18)

if __name__ == "__main__":
    root = tk.Tk()
    app = MapApp(root)
    root.mainloop()
