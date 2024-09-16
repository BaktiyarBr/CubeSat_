import sys
import folium
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
import pandas as pd
from datetime import datetime


class GPSData(QObject):
    new_data = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.gps_data = []

    def add_data(self, data):
        self.gps_data.append(data)
        self.new_data.emit(data)

gps_data = GPSData()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GPS Tracker")

        # Set up the main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Set up the folium map
        self.start_location = [37.7749, -122.4194]  # Initial center of the map
        self.map = folium.Map(location=self.start_location, zoom_start=13)
        self.dataframe = pd.DataFrame(columns=['timestamp', 'latitude', 'longitude', 'altitude'])

        # Save the map to an HTML file
        self.map.save('gps_map.html')

        # Set up the QWebEngineView
        self.web_view = QWebEngineView()
        self.web_view.setHtml(open('gps_map.html').read())
        self.layout.addWidget(self.web_view)

        # Connect the GPSData new_data signal to the update_map method
        gps_data.new_data.connect(self.update_map)

    def update_map(self, data):
        # Update the dataframe with the new data
        self.dataframe = self.dataframe.append(data, ignore_index=True)

        # Create a new folium map
        new_map = folium.Map(location=self.start_location, zoom_start=13)

        # Add GPS points to the map
        for _, row in self.dataframe.iterrows():
            folium.Marker(
                location=[row['latitude'], row['longitude']],
                popup=f"Time: {row['timestamp']}<br>Altitude: {row['altitude']} meters",
                icon=folium.Icon(color='blue', icon='info-sign')
            ).add_to(new_map)

        # Add lines connecting the points (tracks)
        folium.PolyLine(
            locations=[(row['latitude'], row['longitude']) for _, row in self.dataframe.iterrows()],
            color='blue',
            weight=2.5,
            opacity=1
        ).add_to(new_map)

        # Save the new map to an HTML file
        new_map.save('gps_map.html')

        # Update the QWebEngineView
        self.web_view.setHtml(open('gps_map.html').read())

def simulate_gps_data():
    # This function simulates receiving GPS data every 15 seconds
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    new_data = {
        'timestamp': current_time,
        'latitude': 37.7749 + (pd.np.random.rand() - 0.5) * 0.01,
        'longitude': -122.4194 + (pd.np.random.rand() - 0.5) * 0.01,
        'altitude': 10 + pd.np.random.rand() * 10
    }
    gps_data.add_data(new_data)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # Set up a QTimer to simulate receiving GPS data every 15 seconds
    timer = QTimer()
    timer.timeout.connect(simulate_gps_data)
    timer.start(15000)

    sys.exit(app.exec_())
