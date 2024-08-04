import sys
import csv
import os
import math
import json
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QListWidget, QListWidgetItem, QDoubleSpinBox, QLabel, QFileDialog, QCheckBox, QMessageBox
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile, QWebEngineSettings
from PySide6.QtCore import QUrl, QObject, Slot, Qt
from PySide6.QtWebChannel import QWebChannel

class CustomWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        print(f"JS: {message}")

class Bridge(QObject):
    @Slot(float, float)
    def addWaypoint(self, lat, lng):
        window.add_waypoint(lat, lng, is_relative=window.relative_checkbox.isChecked())

    @Slot(int, float, float)
    def updateWaypoint(self, index, lat, lng):
        window.update_waypoint(index, lat, lng)

class WaypointWidget(QWidget):
    def __init__(self, number, lat, lng, throttle=0.2, parent=None):
        super().__init__(parent)
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.number_label = QLabel(f"{number}.")
        self.coords_label = QLabel()
        self.throttle_spinbox = QDoubleSpinBox()
        self.throttle_spinbox.setRange(0, 1)
        self.throttle_spinbox.setSingleStep(0.1)
        self.throttle_spinbox.setValue(throttle)
        self.throttle_spinbox.setDecimals(2)
        self.throttle_spinbox.setFixedWidth(90)  # Set fixed width to 90
        self.throttle_spinbox.setAlignment(Qt.AlignRight)  # Align text to the right

        self.layout.addWidget(self.number_label)
        self.layout.addWidget(self.coords_label, 1)
        self.layout.addWidget(QLabel("Throttle:"))
        self.layout.addWidget(self.throttle_spinbox)

        self.lat = lat
        self.lng = lng
        self.update_coords(lat, lng)

    def update_number(self, number):
        self.number_label.setText(f"{number}.")

    def update_coords(self, lat, lng, relative=False, home_lat=None, home_lng=None):
        self.lat = lat
        self.lng = lng
        if relative and home_lat is not None and home_lng is not None:
            dlat = lat - home_lat
            dlng = lng - home_lng
            dlat_m = dlat * 111139  # Convert degrees latitude to meters
            dlng_m = dlng * 111139 * math.cos(math.radians(home_lat))  # Convert degrees longitude to meters
            self.coords_label.setText(f"ΔLat: {dlat_m:.2f}m, ΔLng: {dlng_m:.2f}m")
        else:
            self.coords_label.setText(f"Lat: {lat:.6f}, Lng: {lng:.6f}")

    def get_data(self):
        text = self.coords_label.text()
        if "ΔLat" in text and "ΔLng" in text:
            dlat_m = float(text.split(", ")[0].split(": ")[1][:-1])
            dlng_m = float(text.split(", ")[1].split(": ")[1][:-1])
            dlat = dlat_m / 111139
            dlng = dlng_m / (111139 * math.cos(math.radians(self.lat)))
            return {
                "lat": self.lat,
                "lng": self.lng,
                "throttle": self.throttle_spinbox.value()
            }
        else:
            return {
                "lat": float(text.split(", ")[0].split(": ")[1]),
                "lng": float(text.split(", ")[1].split(": ")[1]),
                "throttle": self.throttle_spinbox.value()
            }

class WaypointCreator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Donkeycar Waypoint Creator")
        self.setGeometry(100, 100, 1200, 600)

        main_layout = QHBoxLayout()

        self.map_widget = QWebEngineView()
        main_layout.addWidget(self.map_widget, 7)

        sidebar_layout = QVBoxLayout()
        
        self.waypoint_list = QListWidget()
        self.waypoint_list.setDragDropMode(QListWidget.InternalMove)
        self.waypoint_list.model().rowsMoved.connect(self.on_waypoints_reordered)
        sidebar_layout.addWidget(self.waypoint_list)

        delete_button = QPushButton("Delete Selected Waypoint")
        delete_button.clicked.connect(self.delete_selected_waypoint)
        sidebar_layout.addWidget(delete_button)

        save_button = QPushButton("Save Waypoints")
        save_button.clicked.connect(self.save_waypoints)
        sidebar_layout.addWidget(save_button)

        load_button = QPushButton("Load Waypoints")
        load_button.clicked.connect(self.load_waypoints)
        sidebar_layout.addWidget(load_button)

        set_home_button = QPushButton("Set Home to Map Center")
        set_home_button.clicked.connect(self.set_home_to_map_center)
        sidebar_layout.addWidget(set_home_button)

        self.relative_checkbox = QCheckBox("Use Relative Coordinates")
        self.relative_checkbox.stateChanged.connect(self.toggle_relative_coordinates)
        sidebar_layout.addWidget(self.relative_checkbox)

        main_layout.addLayout(sidebar_layout, 3)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.init_map()
        self.home_lat = None
        self.home_lng = None
        self.waypoints = []
        self.is_relative = False

    def init_map(self):
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject('bridge', self.bridge)

        # Set up QWebEngineProfile with relaxed security settings
        profile = QWebEngineProfile.defaultProfile()
        settings = profile.settings()
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        settings.setAttribute(QWebEngineSettings.AllowGeolocationOnInsecureOrigins, True)
        settings.setAttribute(QWebEngineSettings.AllowRunningInsecureContent, True)

        page = CustomWebEnginePage(profile, self.map_widget)
        page.setWebChannel(self.channel)

        # Connect the permission request handler
        page.featurePermissionRequested.connect(self.handlePermissionRequest)

        self.map_widget.setPage(page)

        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_template.html")
        self.map_widget.setUrl(QUrl.fromLocalFile(html_path))

    def handlePermissionRequest(self, url, feature):
        if feature == QWebEnginePage.Geolocation:
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionGrantedByUser)
        else:
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionDeniedByUser)

    def add_waypoint(self, lat, lng, throttle=0.2, is_relative=False):
        if self.home_lat is None:
            self.home_lat, self.home_lng = lat, lng
        
        if is_relative and self.home_lat is not None and self.home_lng is not None:
            # Convert to relative coordinates
            rel_lat = lat - self.home_lat
            rel_lng = lng - self.home_lng
            
            # Convert to meters
            rel_lat_m = rel_lat * 111139
            rel_lng_m = rel_lng * 111139 * math.cos(math.radians(self.home_lat))
            
            waypoint_widget = WaypointWidget(self.waypoint_list.count() + 1, lat, lng, throttle)
            waypoint_widget.update_coords(lat, lng, relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
        else:
            waypoint_widget = WaypointWidget(self.waypoint_list.count() + 1, lat, lng, throttle)
        
        item = QListWidgetItem(self.waypoint_list)
        item.setSizeHint(waypoint_widget.sizeHint())
        self.waypoint_list.addItem(item)
        self.waypoint_list.setItemWidget(item, waypoint_widget)
        
        if is_relative:
            print(f"Relative waypoint added: ΔLat: {rel_lat_m:.2f}m, ΔLng: {rel_lng_m:.2f}m")
        else:
            print(f"Absolute waypoint added: Lat: {lat:.6f}, Lng: {lng:.6f}")
        
        self.update_map_markers()

    def update_waypoint(self, index, lat, lng):
        item = self.waypoint_list.item(index)
        waypoint_widget = self.waypoint_list.itemWidget(item)
        waypoint_widget.update_coords(lat, lng)
        self.update_waypoint_display()
        self.update_map_markers()
        print(f"Waypoint updated: Lat: {lat:.6f}, Lng: {lng:.6f}")

    def on_waypoints_reordered(self):
        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            waypoint_widget.update_number(i + 1)
        self.update_map_markers()

    def delete_selected_waypoint(self):
        selected_items = self.waypoint_list.selectedItems()
        if selected_items:
            for item in selected_items:
                self.waypoint_list.takeItem(self.waypoint_list.row(item))
            self.on_waypoints_reordered()
            self.update_map_markers()

    def update_map_markers(self):
        waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                     for i in range(self.waypoint_list.count())]
        # Ensure throttle is a number, not a string
        for wp in waypoints:
            wp['throttle'] = float(wp['throttle'])
        self.map_widget.page().runJavaScript(f"updateMarkers({json.dumps(waypoints)});")

    def save_waypoints(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Waypoints", "donkey_path.csv", "CSV Files (*.csv)", options=options)
        if file_name:
            waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                         for i in range(self.waypoint_list.count())]
            with open(file_name, 'w', newline='') as f:
                writer = csv.writer(f)
                for i, wp in enumerate(waypoints):
                    if self.relative_checkbox.isChecked() and i > 0:
                        lat, lng = self.get_relative_coords(wp['lat'], wp['lng'])
                    else:
                        lat, lng = wp['lat'], wp['lng']
                    writer.writerow([lat, lng, wp['throttle']])
            print(f"Waypoints saved to {file_name}")

    def load_waypoints(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Waypoints", "", "CSV Files (*.csv)", options=options)
        if file_name:
            self.waypoint_list.clear()
            self.home_lat = None
            self.home_lng = None
            
            self.waypoints = []
            with open(file_name, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) == 3:
                        lat, lng, throttle = float(row[0]), float(row[1]), float(row[2])
                        self.waypoints.append((lat, lng, throttle))
            
            # Detect if coordinates are relative
            self.is_relative = False
            if len(self.waypoints) > 1:
                first_lat, first_lng, _ = self.waypoints[0]
                second_lat, second_lng, _ = self.waypoints[1]
                
                # Check if the second point is within a small range (e.g., ±1 degree)
                if abs(second_lat) < 1 and abs(second_lng) < 1:
                    self.is_relative = True
            
            # Set the checkbox state based on detection
            self.relative_checkbox.setChecked(self.is_relative)
            
            if self.is_relative:
                QMessageBox.information(self, "Set Home Location", "Please set the home location using the 'Set Home to Map Center' button.")
                self.process_absolute_waypoints(self.waypoints)
            else:
                self.process_absolute_waypoints(self.waypoints)
                self.update_map_markers()

    def set_home_to_map_center(self):
        self.map_widget.page().runJavaScript("getMapCenter();", 0, self.on_map_center_received)

    def on_map_center_received(self, result):
        try:
            center = json.loads(result)
            new_home_lat, new_home_lng = center['lat'], center['lng']
            print(f"New home set to: Lat: {new_home_lat:.6f}, Lng: {new_home_lng:.6f}")
            
            if self.relative_checkbox.isChecked():
                self.set_home_and_redraw_waypoints(new_home_lat, new_home_lng)
            else:
                self.home_lat, self.home_lng = new_home_lat, new_home_lng
                
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
            print(f"Received data: {result}")
        except Exception as e:
            print(f"Error processing map center: {e}")

    def set_home_and_redraw_waypoints(self, new_home_lat, new_home_lng):
        if self.home_lat is None or self.home_lng is None:
            self.home_lat, self.home_lng = new_home_lat, new_home_lng
            return

        # Calculate the offset between the old and new home positions
        lat_offset = new_home_lat - self.home_lat
        lng_offset = new_home_lng - self.home_lng

        # Update the home position
        self.home_lat, self.home_lng = new_home_lat, new_home_lng

        # Move all waypoints by the offset
        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            data = waypoint_widget.get_data()
            new_lat = data['lat'] + lat_offset
            new_lng = data['lng'] + lng_offset
            waypoint_widget.update_coords(new_lat, new_lng, relative=True, home_lat=self.home_lat, home_lng=self.home_lng)

        self.update_waypoint_display()
        self.update_map_markers()
        print(f"Waypoints redrawn with new home location.")

    def process_absolute_waypoints(self, waypoints):
        for lat, lng, throttle in waypoints:
            self.add_waypoint(lat, lng, throttle)
        self.update_waypoint_display()
        self.update_map_markers()

    def toggle_relative_coordinates(self):
        self.update_waypoint_display()
        self.update_map_markers()

    def update_waypoint_display(self):
        relative = self.relative_checkbox.isChecked()
        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            data = waypoint_widget.get_data()
            if relative and i > 0:
                waypoint_widget.update_coords(data['lat'], data['lng'], relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
            else:
                waypoint_widget.update_coords(data['lat'], data['lng'])

    def get_relative_coords(self, lat, lng):
        dlat = lat - self.home_lat
        dlng = lng - self.home_lng
        return dlat, dlng

    def get_absolute_coords(self, dlat, dlng):
        lat = self.home_lat + dlat
        lng = self.home_lng + dlng
        return lat, lng

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointCreator()
    window.show()
    sys.exit(app.exec())
