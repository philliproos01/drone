import tkinter as tk
import tkintermapview
import os
from PIL import Image, ImageTk

class MapApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Map Waypoint Collector")
        self.master.geometry("900x600")

        self.waypoints = []
        self.path = None
        self.initial_waypoint = None  # Store the initial waypoint

        # Create a frame for the black box
        self.black_box = tk.Frame(self.master, bg="black", width=112)
        self.black_box.pack(side="left", fill="y")

        # Create a frame for the map and button
        self.right_frame = tk.Frame(self.master)
        self.right_frame.pack(side="right", fill="both", expand=True)

        # Create map widget
        self.map_widget = tkintermapview.TkinterMapView(self.right_frame, width=788, height=550, corner_radius=0)
        self.map_widget.pack(pady=10)

        # Set initial position and zoom
        self.initial_lat, self.initial_lon = 41.672824, -71.441307
        self.map_widget.set_position(self.initial_lat, self.initial_lon)
        self.map_widget.set_zoom(12)

        # Load plane image
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.plane_image = ImageTk.PhotoImage(Image.open(os.path.join(self.current_path, "images", "plane.png")).resize((40, 40)))

        # Add initial waypoint with plane image
        self.add_waypoint((self.initial_lat, self.initial_lon), is_initial=True)

        # Bind click event to map
        self.map_widget.add_left_click_map_command(self.add_waypoint)

        # Create submit button (larger and green)
        self.submit_button = tk.Button(self.right_frame, text="Submit Waypoints", command=self.submit_waypoints,
                                       bg="green", fg="white", font=("Arial", 14), padx=20, pady=10)
        self.submit_button.pack(pady=10)

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
        
        # Clear waypoints after submission
        self.clear_waypoints()

    def marker_callback(self, marker):
        print(marker.text)

    def clear_waypoints(self):
        # Remove all markers
        for marker in self.map_widget.canvas_marker_list:
            self.map_widget.delete_all_marker()

        # Clear the path
        if self.path:
            self.map_widget.delete(self.path)
            self.path = None

        # Reset waypoints list and add back the initial waypoint
        self.waypoints = [(self.initial_lat, self.initial_lon)]
        self.add_waypoint((self.initial_lat, self.initial_lon), is_initial=True)

if __name__ == "__main__":
    root = tk.Tk()
    app = MapApp(root)
    root.mainloop()
