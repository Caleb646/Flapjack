import sys
import json
import numpy as np
from collections import deque
from PyQt5 import QtCore, QtWidgets, QtSerialPort
from PyQt5.QtWidgets import QApplication, QTabWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox
from PyQt5.QtSerialPort import QSerialPort
from pyqtgraph.opengl import GLViewWidget, GLBoxItem, GLGridItem, GLLinePlotItem, GLMeshItem, MeshData
from pyqtgraph import Vector
import pyqtgraph as pg
import trimesh
import os
from datetime import datetime

START_DELIM = b"<"
END_DELIM = b">"

LETTER_X = [
    [(-0.05, -0.05, 0), (0.05, 0.05, 0)],
    [(-0.05, 0.05, 0), (0.05, -0.05, 0)],
]

LETTER_Y = [
    [(-0.05, 0.05, 0), (0, 0, 0)],
    [(0.05, 0.05, 0), (0, 0, 0)],
    [(0, 0, 0), (0, -0.05, 0)],
]

LETTER_Z = [
    [(-0.05, 0.05, 0), (0.05, 0.05, 0)],
    [(0.05, 0.05, 0), (-0.05, -0.05, 0)],
    [(-0.05, -0.05, 0), (0.05, -0.05, 0)],
]

def load_mesh(filename, translate=(0, 0, 0), color=(0.8, 0.8, 0.8, 1.0)):
    mesh = trimesh.load_mesh(filename)
    vertices = mesh.vertices - mesh.centroid
    faces = mesh.faces

    mesh_data = MeshData(vertexes=vertices, faces=faces)
    mesh_item = GLMeshItem(meshdata=mesh_data, color=color, smooth=True) #, drawEdges=True)

    x, y, z = translate
    mesh_item.translate(x, y, z)
    mesh_item.scale(1.0 / mesh.scale, 1.0 / mesh.scale, 1.0 / mesh.scale)  # normalize size
    return mesh_item, mesh.scale

def make_letter(points, offset=(0, 0, 0), color=(1, 1, 1)):
    """Create a GLLinePlotItem shaped like a letter from 3D point pairs."""
    pts = []
    for line in points:
        a = np.array(line[0]) + np.array(offset)
        b = np.array(line[1]) + np.array(offset)
        pts.extend([a, b])
    pts = np.array(pts)
    colors = np.tile(color, (len(pts), 1))
    return GLLinePlotItem(pos=pts, color=colors, width=2, antialias=True)

def make_axis_line(direction, color):
    """
    direction: tuple of 3 floats (x, y, z)
    color: tuple of 3 floats (r, g, b)
    """
    pos = np.array([[0, 0, 0], direction])
    color_array = np.array([color, color])
    return GLLinePlotItem(pos=pos, color=color_array, width=3.0, antialias=True)

class IMUViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Orientation Viewer")
        self.resize(1200, 800)

        # Create main layout
        main_layout = QVBoxLayout(self)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Create 3D viewer tab
        self.viewer_tab = QtWidgets.QWidget()
        self.setup_3d_viewer(self.viewer_tab)
        self.tab_widget.addTab(self.viewer_tab, "3D Orientation")
        
        # Create attitude plotter tab
        self.plotter_tab = AttitudePlotter()
        self.tab_widget.addTab(self.plotter_tab, "Attitude Graphs")
        
        # Add tabs to main layout
        main_layout.addWidget(self.tab_widget)
        
        # Serial controls (shared between tabs)
        self.setup_serial_controls(main_layout)
        
        # Serial port setup
        self.serial = QSerialPort(
            "COM4",
            baudRate=QtSerialPort.QSerialPort.Baud115200,
            readyRead=self.receive
        )
        self.buffer = b""

        # Logging setup
        os.makedirs("./GUI/Logs", exist_ok=True)
        # Wipe out old logs
        self.debug_log = open("./GUI/Logs/debug.log", "w", encoding="utf-8")
        self.flight_data_log = open("./GUI/Logs/flight_data.log", "w", encoding="utf-8")
        self.max_console_lines = 200
        
    def setup_3d_viewer(self, parent_widget):
        """Setup the 3D orientation viewer"""
        layout = QVBoxLayout(parent_widget)
        
        # 3D view setup
        self.view = GLViewWidget()
        self.view.setMinimumHeight(400)
        self.view.opts['distance'] = 5

        grid = GLGridItem()
        grid.scale(2, 2, 1)
        self.view.addItem(grid)

        # Axes (length = 1.0)
        self.x_axis = make_axis_line((0, 1, 0), (1, 0, 0))
        self.y_axis = make_axis_line((1, 0, 0), (0, 1, 0))
        self.z_axis = make_axis_line((0, 0, 1), (0, 0, 1))
        self.view.addItem(self.x_axis)
        self.view.addItem(self.y_axis)
        self.view.addItem(self.z_axis)

        # Labels at tips of each axis
        self.label_x = make_letter(LETTER_X, offset=(0, 1.1, 0), color=(1, 0, 0))
        self.label_y = make_letter(LETTER_Y, offset=(1.1, 0, 0), color=(0, 1, 0))
        self.label_z = make_letter(LETTER_Z, offset=(0, 0, 1.1), color=(0, 0, 1))
        self.view.addItem(self.label_x)
        self.view.addItem(self.label_y)
        self.view.addItem(self.label_z)

        self.airplane, self.airplane_scale = load_mesh("./GUI/data/mesh/plane.stl")
        self.airplane_scale = 1.0 / self.airplane_scale # normalize scale
        self.view.addItem(self.airplane)
        
        layout.addWidget(self.view)
        
    def setup_serial_controls(self, main_layout):
        """Setup serial communication controls"""
        # Serial GUI controls
        self.message_le = QtWidgets.QLineEdit()
        self.send_btn = QtWidgets.QPushButton("Send", clicked=self.send)
        self.output_te = QtWidgets.QTextEdit(readOnly=True)
        self.output_te.setMaximumHeight(150)
        
        # Log level filter controls
        self.log_level_combo = QtWidgets.QComboBox()
        self.log_level_combo.addItems(["[DEBUG]", "[INFO]", "[WARNING]", "[ERROR]"])
        self.log_level_combo.setCurrentText("[INFO]")
        self.log_level_combo.currentTextChanged.connect(self.apply_log_level)

        self.current_log_level = "[INFO]"
        self.button = QtWidgets.QPushButton("Connect", checkable=True, toggled=self.on_toggled)

        # Layout for controls
        top_controls = QHBoxLayout()
        top_controls.addWidget(self.message_le)
        top_controls.addWidget(self.send_btn)
        top_controls.addWidget(self.log_level_combo)
        main_layout.addLayout(top_controls)
        main_layout.addWidget(self.output_te)
        main_layout.addWidget(self.button)

    # def rotate_airplane(self):
    #     # self.rotation_angle += 1  # degrees per frame
    #     if self.rotation_angle >= 360:
    #         self.rotation_angle = 0

    #     # NOTE: Removes scale if the mesh is scaled
    #     # self.airplane.resetTransform()
    #     # Yaw
    #     self.airplane.rotate(self.rotation_angle, 0, 0, 1)
    #     # Pitch
    #     self.airplane.rotate(self.rotation_angle, 0, 1, 0)   
    #     # Roll                     
    #     self.airplane.rotate(self.rotation_angle, 1, 0, 0)                        

    def apply_log_level(self):
        self.current_log_level = self.log_level_combo.currentText()
        self.output_te.clear()  # Optionally clear console on filter change

    def log_level_value(self, level):
        # Map log level string to numeric value for comparison
        levels = {"[DEBUG]": 10, "[INFO]": 20, "[WARNING]": 30, "[ERROR]": 40}
        return levels.get(level.upper(), 20)

    def append_debug_console(self, text, level="[INFO]"):
        # Only show messages at or above the selected log level
        if self.log_level_value(level) >= self.log_level_value(self.current_log_level):
            self.output_te.append(text)
            # Limit the number of lines in the debug console
            doc = self.output_te.document()
            while doc.blockCount() > self.max_console_lines:
                cursor = self.output_te.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.LineUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()  # Remove the newline

    @QtCore.pyqtSlot()
    def receive(self):
        self.buffer += self.serial.readAll().data()
        while START_DELIM in self.buffer and END_DELIM in self.buffer:
            start = self.buffer.find(START_DELIM)
            end = self.buffer.find(END_DELIM, start)
            if end == -1:
                break
            message = self.buffer[start + 1:end]
            self.buffer = self.buffer[end + 1:]
            try:
                data = json.loads(message)
                if data.get("type") == "debug":
                    msg = json.dumps(data)
                    self.debug_log.write(msg + "\n")
                    self.debug_log.flush()
                    self.append_debug_console(
                        msg, level=data.get("lvl", "[INFO]")
                        )
                else:
                    self.flight_data_log.write(json.dumps(data) + "\n")
                    self.flight_data_log.flush()
                    if data.get("type") == "attitude":  # Fixed typo from "attidude"
                        attitude_data = data.get("data", {})
                        self.update_orientation(attitude_data)
                        # Also send to the attitude plotter
                        self.plotter_tab.add_attitude_data(attitude_data)
                    elif data.get("type") == "imu_data":
                        imu_data = data.get("data", {})
                        # Send to the attitude plotter
                        self.plotter_tab.add_imu_data(imu_data)
            except json.JSONDecodeError:
                print(f"Failed to decode JSON: {message}")
                continue

    @QtCore.pyqtSlot()
    def send(self):
        text = self.message_le.text()
        if text:
            self.serial.write(text.encode() + b'\n')

    @QtCore.pyqtSlot(bool)
    def on_toggled(self, checked):
        self.button.setText("Disconnect" if checked else "Connect")
        if checked:
            if not self.serial.isOpen():
                if not self.serial.open(QtCore.QIODevice.ReadWrite):
                    self.output_te.append("Failed to open serial port.")
                    self.button.setChecked(False)
        else:
            self.serial.close()

    def update_orientation(self, orientation):
        roll_deg = orientation.get("roll", 0)
        pitch_deg = orientation.get("pitch", 0)
        yaw_deg = orientation.get("yaw", 0)
 
        self.airplane.resetTransform()
        self.airplane.scale(self.airplane_scale, self.airplane_scale, self.airplane_scale)  # Reset scale
        self.airplane.rotate(yaw_deg, 0, 0, 1)    # Yaw (Z axis)
        self.airplane.rotate(pitch_deg, 0, 1, 0)  # Pitch (Y axis)
        self.airplane.rotate(roll_deg, 1, 0, 0)   # Roll (X axis)

    def closeEvent(self, event):
        # Ensure log files are closed on exit
        try:
            self.debug_log.close()
        except Exception:
            pass
        try:
            self.flight_data_log.close()
        except Exception:
            pass
        super().closeEvent(event)

class AttitudePlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.setup_data()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Control panel
        controls = QHBoxLayout()
        
        # Checkboxes to toggle data series for attitude
        attitude_group = QtWidgets.QGroupBox("Attitude Data")
        attitude_layout = QHBoxLayout(attitude_group)
        
        self.roll_checkbox = QCheckBox("Roll (X)")
        self.roll_checkbox.setChecked(True)
        self.roll_checkbox.stateChanged.connect(self.toggle_roll)
        
        self.pitch_checkbox = QCheckBox("Pitch (Y)")
        self.pitch_checkbox.setChecked(True)
        self.pitch_checkbox.stateChanged.connect(self.toggle_pitch)
        
        self.yaw_checkbox = QCheckBox("Yaw (Z)")
        self.yaw_checkbox.setChecked(True)
        self.yaw_checkbox.stateChanged.connect(self.toggle_yaw)
        
        attitude_layout.addWidget(self.roll_checkbox)
        attitude_layout.addWidget(self.pitch_checkbox)
        attitude_layout.addWidget(self.yaw_checkbox)
        
        # Checkboxes to toggle data series for IMU
        imu_group = QtWidgets.QGroupBox("IMU Data")
        imu_layout = QHBoxLayout(imu_group)
        
        self.ax_checkbox = QCheckBox("Accel X")
        self.ax_checkbox.setChecked(False)
        self.ax_checkbox.stateChanged.connect(self.toggle_ax)
        
        self.ay_checkbox = QCheckBox("Accel Y")
        self.ay_checkbox.setChecked(False)
        self.ay_checkbox.stateChanged.connect(self.toggle_ay)
        
        self.az_checkbox = QCheckBox("Accel Z")
        self.az_checkbox.setChecked(False)
        self.az_checkbox.stateChanged.connect(self.toggle_az)
        
        self.gx_checkbox = QCheckBox("Gyro X")
        self.gx_checkbox.setChecked(False)
        self.gx_checkbox.stateChanged.connect(self.toggle_gx)
        
        self.gy_checkbox = QCheckBox("Gyro Y")
        self.gy_checkbox.setChecked(False)
        self.gy_checkbox.stateChanged.connect(self.toggle_gy)
        
        self.gz_checkbox = QCheckBox("Gyro Z")
        self.gz_checkbox.setChecked(False)
        self.gz_checkbox.stateChanged.connect(self.toggle_gz)
        
        imu_layout.addWidget(self.ax_checkbox)
        imu_layout.addWidget(self.ay_checkbox)
        imu_layout.addWidget(self.az_checkbox)
        imu_layout.addWidget(self.gx_checkbox)
        imu_layout.addWidget(self.gy_checkbox)
        imu_layout.addWidget(self.gz_checkbox)
        
        # Control buttons
        buttons_group = QtWidgets.QGroupBox("Controls")
        buttons_layout = QHBoxLayout(buttons_group)
        
        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        
        self.load_btn = QPushButton("Load from File")
        self.load_btn.clicked.connect(self.load_from_file)
        
        buttons_layout.addWidget(self.clear_btn)
        buttons_layout.addWidget(self.load_btn)
        
        controls.addWidget(attitude_group)
        controls.addWidget(imu_group)
        controls.addWidget(buttons_group)
        controls.addStretch()
        
        layout.addLayout(controls)
        
        # Create the plot widgets
        # Attitude plot
        self.attitude_widget = pg.PlotWidget()
        self.attitude_widget.setLabel('left', 'Angle (degrees)')
        self.attitude_widget.setLabel('bottom', 'Time (seconds)')
        self.attitude_widget.setTitle('Attitude Data (Roll, Pitch, Yaw)')
        self.attitude_widget.addLegend()
        self.attitude_widget.showGrid(x=True, y=True)
        
        # IMU Accelerometer plot
        self.accel_widget = pg.PlotWidget()
        self.accel_widget.setLabel('left', 'Acceleration (m/s²)')
        self.accel_widget.setLabel('bottom', 'Time (seconds)')
        self.accel_widget.setTitle('Accelerometer Data (X, Y, Z)')
        self.accel_widget.addLegend()
        self.accel_widget.showGrid(x=True, y=True)
        
        # IMU Gyroscope plot
        self.gyro_widget = pg.PlotWidget()
        self.gyro_widget.setLabel('left', 'Angular Velocity (°/s)')
        self.gyro_widget.setLabel('bottom', 'Time (seconds)')
        self.gyro_widget.setTitle('Gyroscope Data (X, Y, Z)')
        self.gyro_widget.addLegend()
        self.gyro_widget.showGrid(x=True, y=True)
        
        layout.addWidget(self.attitude_widget)
        layout.addWidget(self.accel_widget)
        layout.addWidget(self.gyro_widget)
        
    def setup_data(self):
        # Data storage - using deque for efficient append/pop operations
        self.max_points = 1000  # Maximum number of points to display
        self.start_time = None
        
        # Time data (shared across all plots)
        self.time_data = deque(maxlen=self.max_points)
        
        # Attitude data
        self.roll_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.yaw_data = deque(maxlen=self.max_points)
        
        # IMU accelerometer data (in m/s²)
        self.ax_data = deque(maxlen=self.max_points)
        self.ay_data = deque(maxlen=self.max_points)
        self.az_data = deque(maxlen=self.max_points)
        
        # IMU gyroscope data (in °/s)
        self.gx_data = deque(maxlen=self.max_points)
        self.gy_data = deque(maxlen=self.max_points)
        self.gz_data = deque(maxlen=self.max_points)
        
        # Attitude plot curves
        self.roll_curve = self.attitude_widget.plot(
            pen=pg.mkPen(color='r', width=2), 
            name='Roll (X)'
        )
        self.pitch_curve = self.attitude_widget.plot(
            pen=pg.mkPen(color='g', width=2), 
            name='Pitch (Y)'
        )
        self.yaw_curve = self.attitude_widget.plot(
            pen=pg.mkPen(color='b', width=2), 
            name='Yaw (Z)'
        )
        
        # Accelerometer plot curves
        self.ax_curve = self.accel_widget.plot(
            pen=pg.mkPen(color='r', width=2), 
            name='Accel X'
        )
        self.ay_curve = self.accel_widget.plot(
            pen=pg.mkPen(color='g', width=2), 
            name='Accel Y'
        )
        self.az_curve = self.accel_widget.plot(
            pen=pg.mkPen(color='b', width=2), 
            name='Accel Z'
        )
        
        # Gyroscope plot curves
        self.gx_curve = self.gyro_widget.plot(
            pen=pg.mkPen(color='r', width=2), 
            name='Gyro X'
        )
        self.gy_curve = self.gyro_widget.plot(
            pen=pg.mkPen(color='g', width=2), 
            name='Gyro Y'
        )
        self.gz_curve = self.gyro_widget.plot(
            pen=pg.mkPen(color='b', width=2), 
            name='Gyro Z'
        )
        
    def add_attitude_data(self, attitude_data):
        """Add new attitude data point"""
        current_time = datetime.now()
        
        if self.start_time is None:
            self.start_time = current_time
            
        # Calculate time relative to start
        time_seconds = (current_time - self.start_time).total_seconds()
        
        # Extract attitude values
        roll = attitude_data.get("roll", 0)
        pitch = attitude_data.get("pitch", 0)
        yaw = attitude_data.get("yaw", 0)
        
        # Add data points
        self.time_data.append(time_seconds)
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        self.yaw_data.append(yaw)
        
        # Add current IMU data if available, otherwise use last values or zeros
        if len(self.ax_data) > 0:
            self.ax_data.append(self.ax_data[-1])
            self.ay_data.append(self.ay_data[-1])
            self.az_data.append(self.az_data[-1])
            self.gx_data.append(self.gx_data[-1])
            self.gy_data.append(self.gy_data[-1])
            self.gz_data.append(self.gz_data[-1])
        else:
            self.ax_data.append(0)
            self.ay_data.append(0)
            self.az_data.append(0)
            self.gx_data.append(0)
            self.gy_data.append(0)
            self.gz_data.append(0)
        
        # Update plots
        self.update_plots()
        
    def add_imu_data(self, imu_data):
        """Add new IMU data point"""
        current_time = datetime.now()
        
        if self.start_time is None:
            self.start_time = current_time
            
        # Calculate time relative to start
        time_seconds = (current_time - self.start_time).total_seconds()
        
        # Extract IMU values (convert from int16 back to actual values)
        ax = imu_data.get("ax", 0)
        ay = imu_data.get("ay", 0)
        az = imu_data.get("az", 0)
        gx = imu_data.get("gx", 0)  
        gy = imu_data.get("gy", 0)
        gz = imu_data.get("gz", 0)
        
        # Add data points
        self.time_data.append(time_seconds)
        self.ax_data.append(ax / 1000.0)
        self.ay_data.append(ay / 1000.0)
        self.az_data.append(az / 1000.0)
        self.gx_data.append(gx / 1000.0)  # Convert back to °/s
        self.gy_data.append(gy / 1000.0)
        self.gz_data.append(gz / 1000.0)
        
        # Add current attitude data if available, otherwise use last values or zeros
        if len(self.roll_data) > 0:
            self.roll_data.append(self.roll_data[-1])
            self.pitch_data.append(self.pitch_data[-1])
            self.yaw_data.append(self.yaw_data[-1])
        else:
            self.roll_data.append(0)
            self.pitch_data.append(0)
            self.yaw_data.append(0)
        
        # Update plots
        self.update_plots()
        
    def update_plots(self):
        """Update the plot curves with current data"""
        time_array = np.array(self.time_data)
        
        # Update attitude plots
        if self.roll_checkbox.isChecked():
            self.roll_curve.setData(time_array, np.array(self.roll_data))
        
        if self.pitch_checkbox.isChecked():
            self.pitch_curve.setData(time_array, np.array(self.pitch_data))
            
        if self.yaw_checkbox.isChecked():
            self.yaw_curve.setData(time_array, np.array(self.yaw_data))
            
        # Update accelerometer plots
        if self.ax_checkbox.isChecked():
            self.ax_curve.setData(time_array, np.array(self.ax_data))
            
        if self.ay_checkbox.isChecked():
            self.ay_curve.setData(time_array, np.array(self.ay_data))
            
        if self.az_checkbox.isChecked():
            self.az_curve.setData(time_array, np.array(self.az_data))
            
        # Update gyroscope plots
        if self.gx_checkbox.isChecked():
            self.gx_curve.setData(time_array, np.array(self.gx_data))
            
        if self.gy_checkbox.isChecked():
            self.gy_curve.setData(time_array, np.array(self.gy_data))
            
        if self.gz_checkbox.isChecked():
            self.gz_curve.setData(time_array, np.array(self.gz_data))
            
    def toggle_roll(self, state):
        """Toggle roll data visibility"""
        if state:
            self.roll_curve.setData(np.array(self.time_data), np.array(self.roll_data))
        else:
            self.roll_curve.setData([], [])
            
    def toggle_pitch(self, state):
        """Toggle pitch data visibility"""
        if state:
            self.pitch_curve.setData(np.array(self.time_data), np.array(self.pitch_data))
        else:
            self.pitch_curve.setData([], [])
            
    def toggle_yaw(self, state):
        """Toggle yaw data visibility"""
        if state:
            self.yaw_curve.setData(np.array(self.time_data), np.array(self.yaw_data))
        else:
            self.yaw_curve.setData([], [])
            
    def toggle_ax(self, state):
        """Toggle accelerometer X data visibility"""
        if state:
            self.ax_curve.setData(np.array(self.time_data), np.array(self.ax_data))
        else:
            self.ax_curve.setData([], [])
            
    def toggle_ay(self, state):
        """Toggle accelerometer Y data visibility"""
        if state:
            self.ay_curve.setData(np.array(self.time_data), np.array(self.ay_data))
        else:
            self.ay_curve.setData([], [])
            
    def toggle_az(self, state):
        """Toggle accelerometer Z data visibility"""
        if state:
            self.az_curve.setData(np.array(self.time_data), np.array(self.az_data))
        else:
            self.az_curve.setData([], [])
            
    def toggle_gx(self, state):
        """Toggle gyroscope X data visibility"""
        if state:
            self.gx_curve.setData(np.array(self.time_data), np.array(self.gx_data))
        else:
            self.gx_curve.setData([], [])
            
    def toggle_gy(self, state):
        """Toggle gyroscope Y data visibility"""
        if state:
            self.gy_curve.setData(np.array(self.time_data), np.array(self.gy_data))
        else:
            self.gy_curve.setData([], [])
            
    def toggle_gz(self, state):
        """Toggle gyroscope Z data visibility"""
        if state:
            self.gz_curve.setData(np.array(self.time_data), np.array(self.gz_data))
        else:
            self.gz_curve.setData([], [])
            
    def clear_data(self):
        """Clear all attitude and IMU data"""
        self.time_data.clear()
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()
        self.ax_data.clear()
        self.ay_data.clear()
        self.az_data.clear()
        self.gx_data.clear()
        self.gy_data.clear()
        self.gz_data.clear()
        self.start_time = None
        
        # Clear plots
        self.roll_curve.setData([], [])
        self.pitch_curve.setData([], [])
        self.yaw_curve.setData([], [])
        self.ax_curve.setData([], [])
        self.ay_curve.setData([], [])
        self.az_curve.setData([], [])
        self.gx_curve.setData([], [])
        self.gy_curve.setData([], [])
        self.gz_curve.setData([], [])
        
    def load_from_file(self):
        """Load attitude and IMU data from flight_data.log file"""
        try:
            log_file_path = "./GUI/Logs/flight_data.log"
            if not os.path.exists(log_file_path):
                QtWidgets.QMessageBox.warning(self, "Warning", "No flight data log file found!")
                return
                
            self.clear_data()
            
            with open(log_file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                
            if not lines:
                QtWidgets.QMessageBox.information(self, "Info", "No data found in log file!")
                return
                
            # Process each line
            attitude_count = 0
            imu_count = 0
            
            for i, line in enumerate(lines):
                try:
                    data = json.loads(line.strip())
                    
                    if data.get("type") == "attitude":
                        attitude_data = data.get("data", {})
                        
                        # Use line number as time for file data (since we don't have timestamps)
                        time_seconds = i * 0.1  # Assume 10Hz data rate
                        
                        roll = attitude_data.get("roll", 0)
                        pitch = attitude_data.get("pitch", 0)
                        yaw = attitude_data.get("yaw", 0)
                        
                        self.time_data.append(time_seconds)
                        self.roll_data.append(roll)
                        self.pitch_data.append(pitch)
                        self.yaw_data.append(yaw)
                        
                        # Add placeholder IMU data if not present
                        self.ax_data.append(0)
                        self.ay_data.append(0)
                        self.az_data.append(0)
                        self.gx_data.append(0)
                        self.gy_data.append(0)
                        self.gz_data.append(0)
                        
                        attitude_count += 1
                        
                    elif data.get("type") == "imu_data":
                        imu_data = data.get("data", {})
                        
                        # Use line number as time for file data
                        time_seconds = i * 0.1  # Assume 10Hz data rate
                        
                        # Extract IMU values
                        ax = imu_data.get("ax", 0)
                        ay = imu_data.get("ay", 0)
                        az = imu_data.get("az", 0)
                        gx = imu_data.get("gx", 0)
                        gy = imu_data.get("gy", 0)
                        gz = imu_data.get("gz", 0)
                        
                        self.time_data.append(time_seconds)
                        self.ax_data.append(ax)
                        self.ay_data.append(ay)
                        self.az_data.append(az)
                        self.gx_data.append(gx / 1000.0)  # Convert to °/s
                        self.gy_data.append(gy / 1000.0)
                        self.gz_data.append(gz / 1000.0)
                        
                        # Add placeholder attitude data if not present
                        self.roll_data.append(0)
                        self.pitch_data.append(0)
                        self.yaw_data.append(0)
                        
                        imu_count += 1
                        
                except json.JSONDecodeError:
                    continue  # Skip invalid JSON lines
                    
            self.update_plots()
            QtWidgets.QMessageBox.information(self, "Success", 
                f"Loaded {attitude_count} attitude and {imu_count} IMU data points!")
            
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load data: {str(e)}")

# ...existing letter and mesh functions...
def main():
    app = QApplication(sys.argv)
    viewer = IMUViewer()
    viewer.show()
    # Ensure log files are closed on app exit
    app.aboutToQuit.connect(viewer.close)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
