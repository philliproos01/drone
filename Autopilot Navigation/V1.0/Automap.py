import sys
import io
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import folium
from folium.plugins import MousePosition

class MapWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.waypoints = []
        self.webView = None  # Initialize webView as None
        self.initUI()

    def initUI(self):
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Create the black box on the left
        left_panel = QFrame()
        left_panel.setStyleSheet("background-color: black;")
        left_panel.setFixedWidth(80)
        main_layout.addWidget(left_panel)

        # Create a vertical layout for the map, title, and submit button
        right_layout = QVBoxLayout()
        main_layout.addLayout(right_layout)

        # Create the "MAP" text
        title_label = QLabel("MAP")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: black;")
        title_label.setFont(QFont("Arial", 24, QFont.Bold))
        title_label.setFixedHeight(60)
        right_layout.addWidget(title_label)

        # Create the webView
        self.webView = QWebEngineView()
        right_layout.addWidget(self.webView)

        # Create the map
        self.coordinate = [41.808425, -70.957847]
        self.create_map()

        # Create the submit button
        submit_button = QPushButton("Submit")
        submit_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                border: none;
                color: white;
                padding: 15px 32px;
                text-align: center;
                text-decoration: none;
                font-size: 16px;
                margin: 4px 2px;
                cursor: pointer;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        submit_button.clicked.connect(self.on_submit)
        right_layout.addWidget(submit_button)

        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Folium Map in PyQt5')
        self.show()

    def create_map(self):
        m = folium.Map(location=self.coordinate, zoom_start=14)
        folium.Marker(self.coordinate, popup="Specified Location").add_to(m)

        # Add mouse position display
        MousePosition().add_to(m)

        # Add click event listener
        m.add_child(folium.LatLngPopup())

        folium.TileLayer('openstreetmap').add_to(m)

        # Add existing waypoints
        for i, waypoint in enumerate(self.waypoints, 1):
            folium.Marker(
                waypoint,
                popup=f'Waypoint {i}',
                icon=folium.Icon(color='red', icon='info-sign')
            ).add_to(m)

        # Add click event to add waypoint
        m.add_child(folium.ClickForMarker(popup='Waypoint'))

        data = io.BytesIO()
        m.save(data, close_file=False)
        self.webView.setHtml(data.getvalue().decode())

    def add_waypoint(self, lat, lng):
        self.waypoints.append((lat, lng))
        self.create_map()  # Recreate the map with the new waypoint
        print(f"Waypoint added: {lat}, {lng}")

    def on_submit(self):
        print("Waypoints:")
        for i, waypoint in enumerate(self.waypoints, 1):
            print(f"Waypoint {i}: Latitude {waypoint[0]}, Longitude {waypoint[1]}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MapWindow()
    sys.exit(app.exec_())
