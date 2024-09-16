import sys
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QPushButton, QComboBox
from PyQt5.QtOpenGL import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import numpy as np
import threading

class MyOpenGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super(MyOpenGLWidget, self).__init__(parent)
        self.ax1 = self.ay1 = self.az1 = 0.0
        self.yaw_mode = False

    def initializeGL(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def resizeGL(self, w, h):
        if h == 0:
            h = 1
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def paintGL(self):
        self.draw()

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -7.0)

        osd_text = f"pitch: {self.ay1:.2f}, roll: {self.ax1:.2f}"
        if self.yaw_mode:
            osd_text += f", yaw: {self.az1:.2f}"

        self.drawText((-2, -2, 2), osd_text)

        if self.yaw_mode:
            glRotatef(self.az1, 0.0, 1.0, 0.0)
        glRotatef(self.ay1, 1.0, 0.0, 0.0)
        glRotatef(-self.ax1, 0.0, 0.0, 1.0)

        glBegin(GL_QUADS)
        self.draw_cube()
        glEnd()

    def drawText(self, position, textString):
        # This method needs to be updated to render text within a QOpenGLWidget
        pass

    def draw_cube(self):
        vertices = [
            [1.0, 0.2, -1.0], [-1.0, 0.2, -1.0], [-1.0, 0.2, 1.0], [1.0, 0.2, 1.0],
            [1.0, -0.2, 1.0], [-1.0, -0.2, 1.0], [-1.0, -0.2, -1.0], [1.0, -0.2, -1.0]
        ]
        colors = [
            [0.0, 1.0, 0.0], [1.0, 0.5, 0.0], [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0]
        ]
        indices = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 3, 4, 7), (1, 2, 5, 6), (0, 1, 6, 7), (2, 3, 4, 5)]
        for face, color in zip(indices, colors):
            glColor3fv(color)
            for vertex in face:
                glVertex3fv(vertices[vertex])

    def update_data(self, roll, pitch, yaw):
        self.ax1 = roll
        self.ay1 = pitch
        self.az1 = yaw
        self.update()  # Triggers the repaint event

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Visualization")

        # Create tab widget and add tabs
        self.tabWidget = QtWidgets.QTabWidget(self)
        self.setCentralWidget(self.tabWidget)

        self.tabIMU = QtWidgets.QWidget()
        self.tabIMU.setObjectName("tabIMU")

        # Create and add OpenGL widget
        self.openGLWidget_IMU = MyOpenGLWidget(self.tabIMU)
        self.openGLWidget_IMU.setGeometry(QtCore.QRect(170, 10, 611, 501))
        self.openGLWidget_IMU.setObjectName("openGLWidget_IMU")

        # Create and add buttons and combo box
        self.pushButtonScan_IMU = QtWidgets.QPushButton(self.tabIMU)
        self.pushButtonScan_IMU.setGeometry(QtCore.QRect(10, 20, 75, 24))
        self.pushButtonScan_IMU.setObjectName("pushButtonScan_IMU")
        self.pushButtonScan_IMU.setText("Scan")

        self.pushButtonConnect_IMU = QtWidgets.QPushButton(self.tabIMU)
        self.pushButtonConnect_IMU.setGeometry(QtCore.QRect(10, 50, 75, 24))
        self.pushButtonConnect_IMU.setObjectName("pushButtonConnect_IMU")
        self.pushButtonConnect_IMU.setText("Connect")

        self.comboBox_ComPorts_IMU = QtWidgets.QComboBox(self.tabIMU)
        self.comboBox_ComPorts_IMU.setGeometry(QtCore.QRect(10, 110, 100, 22))
        self.comboBox_ComPorts_IMU.setObjectName("comboBox_ComPorts_IMU")

        self.tabWidget.addTab(self.tabIMU, "IMU")

        # Start serial reading thread
        self.start_serial_reading()

    def start_serial_reading(self):
        port = 'COM6'
        baudrate = 115200

        def read_from_com_port():
            ser = serial.Serial(port, baudrate, timeout=1)
            dt = 0.1
            process_noise = 0.01
            measurement_noise = 0.1

            roll_kf = KalmanFilter(dt, process_noise, measurement_noise)
            pitch_kf = KalmanFilter(dt, process_noise, measurement_noise)
            yaw_kf = KalmanFilter(dt, process_noise, measurement_noise)

            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8', errors='replace').strip()
                    if "Acceleration" in data or "Gyroscope" in data:
                        parsed_data = parse_sensor_data(data)
                        if parsed_data:
                            ax, ay, az = parsed_data["Acceleration"]
                            gx, gy, gz = parsed_data["Gyroscope"]

                            roll_acc = np.arctan2(ay, az) * 180 / np.pi
                            pitch_acc = np.arctan2(-ax, np.sqrt(ay ** 2 + az ** 2)) * 180 / np.pi
                            yaw_acc = 0

                            gx, gy = np.radians([gx, gy])

                            roll_kf.predict(gx)
                            roll_kf.update(roll_acc)
                            pitch_kf.predict(gy)
                            pitch_kf.update(pitch_acc)
                            yaw_kf.predict(gz)
                            yaw_kf.update(yaw_acc)

                            roll = roll_kf.get_angle()
                            pitch = pitch_kf.get_angle()
                            yaw = yaw_kf.get_angle()

                            self.openGLWidget_IMU.update_data(roll, pitch, yaw)

        threading.Thread(target=read_from_com_port, daemon=True).start()

class KalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise):
        self.dt = dt
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.x = np.zeros((2, 1))
        self.P = np.eye(2)
        self.F = np.array([[1, -dt], [0, 1]])
        self.B = np.array([[dt], [0]])
        self.H = np.array([[1, 0]])
        self.Q = process_noise * np.eye(2)
        self.R = np.array([[measurement_noise]])

    def predict(self, u):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x += np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = (I - np.dot(K, self.H)) @ self.P

    def get_angle(self):
        return self.x[0, 0]

def parse_sensor_data(data):
    try:
        parts = data.split(',')
        if len(parts) != 8:
            return None
        sensor_type_1 = parts[0].strip()
        ax, ay, az = map(float, parts[1:4])
        sensor_type_2 = parts[4].strip()
        gx, gy, gz = map(float, parts[5:8])
        if sensor_type_1 == "Acceleration" and sensor_type_2 == "Gyroscope":
            return {"Acceleration": [ax, ay, az], "Gyroscope": [gx, gy, gz]}
    except ValueError:
        return None
    return None

def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
