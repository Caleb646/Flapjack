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
from datetime import datetime
import trimesh
import os
import struct

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

class FlightViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flight Viewer")
        self.resize(1200, 800)

        main_layout = QVBoxLayout(self)

        self.tab_widget = QTabWidget()
        
        self.viewer_tab = QtWidgets.QWidget()
        self.setup_3d_viewer(self.viewer_tab)
        self.tab_widget.addTab(self.viewer_tab, "3D Orientation")
        
        self.plotter_tab = AttitudePlotter()
        self.tab_widget.addTab(self.plotter_tab, "Attitude Graphs")
        
        self.actuator_tab = ActuatorPlotter()
        self.tab_widget.addTab(self.actuator_tab, "Actuator Graphs")
        
        self.control_tab = FlightControlTab(self)
        self.tab_widget.addTab(self.control_tab, "Flight Control")
        
        main_layout.addWidget(self.tab_widget)

        self.setup_serial_controls(main_layout)
        
        self.serial = QSerialPort(
            "COM4",
            # baudRate=QtSerialPort.QSerialPort.Baud115200,
            readyRead=self.receive
        )
        if not self.serial.setBaudRate(230400, QtSerialPort.QSerialPort.AllDirections):
            raise RuntimeError("Could not set baud rate for serial port")
        self.buffer = b""

        os.makedirs("./GUI/Logs", exist_ok=True)  
        self.debug_log = None
        self.flight_data_log = None
        self.current_session_folder = None
        self.max_console_lines = 200
        
    def setup_3d_viewer(self, parent_widget):
        layout = QVBoxLayout(parent_widget)
        
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
        self.message_le = QtWidgets.QLineEdit()
        self.send_btn = QtWidgets.QPushButton("Send", clicked=self.send)
        self.output_te = QtWidgets.QTextEdit(readOnly=True)
        self.output_te.setMaximumHeight(150)
        
        self.log_level_combo = QtWidgets.QComboBox()
        self.log_level_combo.addItems(["[DEBUG]", "[INFO]", "[WARNING]", "[ERROR]"])
        self.log_level_combo.setCurrentText("[INFO]")
        self.log_level_combo.currentTextChanged.connect(self.apply_log_level)

        self.current_log_level = "[INFO]"
        self.button = QtWidgets.QPushButton("Connect", checkable=True, toggled=self.on_toggled)

        top_controls = QHBoxLayout()
        top_controls.addWidget(self.message_le)
        top_controls.addWidget(self.send_btn)
        top_controls.addWidget(self.log_level_combo)
        main_layout.addLayout(top_controls)
        main_layout.addWidget(self.output_te)
        main_layout.addWidget(self.button)                    

    def apply_log_level(self):
        self.current_log_level = self.log_level_combo.currentText()
        self.output_te.clear()

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
                    if self.debug_log:
                        self.debug_log.write(msg + "\n")
                        self.debug_log.flush()
                    self.append_debug_console(
                        msg, level=data.get("lvl", "[INFO]")
                        )
                else:
                    if self.flight_data_log: 
                        self.flight_data_log.write(json.dumps(data) + "\n")
                        self.flight_data_log.flush()
                    if data.get("type") == "attitude":  # Fixed typo from "attidude"
                        attitude_data = data.get("data", {})
                        self.update_orientation(attitude_data)
                        self.plotter_tab.add_attitude_data(attitude_data)
                    elif data.get("type") == "imu_data":
                        imu_data = data.get("data", {})
                        self.plotter_tab.add_imu_data(imu_data)
                    elif data.get("type") == "actuators":
                        actuator_data = data.get("data", {})
                        self.actuator_tab.add_actuator_data(actuator_data)
            except json.JSONDecodeError:
                print(f"Failed to decode JSON: {message}")
                continue

    @QtCore.pyqtSlot()
    def send(self):
        text = self.message_le.text()
        if text:
            self.serial.write(text.encode() + b'\n')

    def send_start_command(self):
        """Send start command as 8-byte packet"""
        if not self.serial.isOpen():
            self.append_debug_console("Serial port not connected!", "[ERROR]")
            return
            
        # Command structure based on C code:
        # typedef struct {
        #     CommandHeader header;
        #     eREQUESTED_STATE_t requestedState;
        # } ChangeOpStateCmd;
        #
        # CommandHeader: { uint8_t commandType; }
        # eREQUESTED_STATE_t: uint8_t
        
        COMMAND_TYPE_CHANGE_OP_STATE = 1
        REQUESTED_STATE_START = 1        
        
        # Create 8-byte packet: commandType(1) + requestedState(1) + padding(6)
        # B = uint8_t and 6x = 6 bytes of padding
        packet = struct.pack('<BB6x', COMMAND_TYPE_CHANGE_OP_STATE, REQUESTED_STATE_START)
        
        try:
            self.serial.write(packet)
            self.append_debug_console(f"Sent START command: {packet.hex()}", "[INFO]")
        except Exception as e:
            self.append_debug_console(f"Failed to send START command: {e}", "[ERROR]")

    def send_stop_command(self):
        """Send stop command as 8-byte packet"""
        if not self.serial.isOpen():
            self.append_debug_console("Serial port not connected!", "[ERROR]")
            return
        
        COMMAND_TYPE_CHANGE_OP_STATE = 1
        REQUESTED_STATE_STOP = 0        
        
        # Create 8-byte packet: commandType(1) + requestedState(1) + padding(6)
        packet = struct.pack('<BB6x', COMMAND_TYPE_CHANGE_OP_STATE, REQUESTED_STATE_STOP)
        
        try:
            self.serial.write(packet)
            self.append_debug_console(f"Sent STOP command: {packet.hex()}", "[INFO]")
        except Exception as e:
            self.append_debug_console(f"Failed to send STOP command: {e}", "[ERROR]")

    @QtCore.pyqtSlot(bool)
    def on_toggled(self, checked):
        self.button.setText("Disconnect" if checked else "Connect")
        if checked:
            if not self.serial.isOpen():
                self.create_session_folder()
                if not self.serial.open(QtCore.QIODevice.ReadWrite):
                    self.output_te.append("Failed to open serial port.")
                    self.button.setChecked(False)
                    self.close_session_files()
                else:
                    # Enable command buttons when connected
                    self.control_tab.set_buttons_enabled(True)
                    self.append_debug_console("Connected to flight controller", "[INFO]")
        else:
            # NOTE: Send stop command when disconnecting the GUI
            self.send_stop_command()
            # Make sure the stop command is sent before closing
            self.serial.flush()

            self.close_session_files()
            self.serial.close()
            # Disable command buttons when disconnected
            self.control_tab.set_buttons_enabled(False)
            self.append_debug_console("Disconnected from flight controller", "[INFO]")

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
        # Ensure session files are properly closed on exit
        self.close_session_files()
        super().closeEvent(event)

    def create_session_folder(self):
        """Create a timestamped folder for the current session and open log files"""
        # Create timestamp in format: Month_Day_Hour_Minutes_Seconds
        timestamp = datetime.now().strftime("%m_%d_%H_%M_%S")
        self.current_session_folder = f"./GUI/Logs/Session_{timestamp}"
        
        # Create the session directory
        os.makedirs(self.current_session_folder, exist_ok=True)
        
        # Open new log files in the session folder
        debug_path = os.path.join(self.current_session_folder, "debug.log")
        flight_data_path = os.path.join(self.current_session_folder, "flight_data.log")
        
        self.debug_log = open(debug_path, "w", encoding="utf-8")
        self.flight_data_log = open(flight_data_path, "w", encoding="utf-8")
        
        # Log session start
        session_start_msg = f"Session started at {datetime.now().isoformat()}"
        self.debug_log.write(f"{{\"type\":\"debug\",\"lvl\":\"[INFO]\",\"msg\":\"{session_start_msg}\"}}\n")
        self.debug_log.flush()
        
        self.append_debug_console(f"Created session folder: {self.current_session_folder}", "[INFO]")
    
    def close_session_files(self):
        """Close log files and flush any remaining data"""
        if self.debug_log:
            try:
                # Log session end
                session_end_msg = f"Session ended at {datetime.now().isoformat()}"
                self.debug_log.write(f"{{\"type\":\"debug\",\"lvl\":\"[INFO]\",\"msg\":\"{session_end_msg}\"}}\n")
                self.debug_log.flush()
                self.debug_log.close()
                self.debug_log = None
            except Exception as e:
                print(f"Error closing debug log: {e}")
        
        if self.flight_data_log:
            try:
                self.flight_data_log.flush()
                self.flight_data_log.close()
                self.flight_data_log = None
            except Exception as e:
                print(f"Error closing flight data log: {e}")
        
        if self.current_session_folder:
            self.append_debug_console(f"Session files saved to: {self.current_session_folder}", "[INFO]")
            self.current_session_folder = None

class FlightControlTab(QtWidgets.QWidget):
    def __init__(self, parent_viewer):
        super().__init__()
        self.parent_viewer = parent_viewer
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Flight Controller Commands
        flight_control_group = QtWidgets.QGroupBox("Flight Controller Commands")
        flight_control_layout = QHBoxLayout(flight_control_group)
        
        self.start_btn = QtWidgets.QPushButton("Start", clicked=self.send_start_command)
        self.start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        self.start_btn.setEnabled(False)  # Disabled until connected
        
        self.stop_btn = QtWidgets.QPushButton("Stop", clicked=self.send_stop_command)
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 10px; }")
        self.stop_btn.setEnabled(False)  # Disabled until connected
        
        flight_control_layout.addWidget(self.start_btn)
        flight_control_layout.addWidget(self.stop_btn)
        
        layout.addWidget(flight_control_group)
        
        # Status display
        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; }")
        
        self.last_command_label = QtWidgets.QLabel("Last Command: None")
        self.last_command_label.setStyleSheet("QLabel { padding: 5px; }")
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.last_command_label)
        
        layout.addWidget(status_group)
        
        # Add spacer to push everything to the top
        layout.addStretch()
        
    def set_buttons_enabled(self, enabled):
        """Enable or disable command buttons"""
        self.start_btn.setEnabled(enabled)
        self.stop_btn.setEnabled(enabled)
        
        if enabled:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; color: green; }")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("QLabel { font-weight: bold; padding: 5px; color: red; }")
            
    def send_start_command(self):
        """Send start command as 8-byte packet"""
        self.parent_viewer.send_start_command()
        self.last_command_label.setText("Last Command: START")
        
    def send_stop_command(self):
        """Send stop command as 8-byte packet"""
        self.parent_viewer.send_stop_command()
        self.last_command_label.setText("Last Command: STOP")

class ActuatorPlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.setup_data()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Control panel
        controls = QHBoxLayout()
        
        # Checkboxes to toggle data series for motors
        motor_group = QtWidgets.QGroupBox("Motor Data")
        motor_layout = QHBoxLayout(motor_group)
        
        self.motor_throttle_checkbox = QCheckBox("Motor Throttle")
        self.motor_throttle_checkbox.setChecked(True)
        self.motor_throttle_checkbox.stateChanged.connect(self.toggle_motor_throttle)
        
        self.motor_target_throttle_checkbox = QCheckBox("Motor Target Throttle")
        self.motor_target_throttle_checkbox.setChecked(True)
        self.motor_target_throttle_checkbox.stateChanged.connect(self.toggle_motor_target_throttle)
        
        motor_layout.addWidget(self.motor_throttle_checkbox)
        motor_layout.addWidget(self.motor_target_throttle_checkbox)
        
        # Checkboxes to toggle data series for servos
        servo_group = QtWidgets.QGroupBox("Servo Data")
        servo_layout = QHBoxLayout(servo_group)
        
        self.servo_angle_checkbox = QCheckBox("Servo Angle")
        self.servo_angle_checkbox.setChecked(True)
        self.servo_angle_checkbox.stateChanged.connect(self.toggle_servo_angle)
        
        self.servo_target_angle_checkbox = QCheckBox("Servo Target Angle")
        self.servo_target_angle_checkbox.setChecked(True)
        self.servo_target_angle_checkbox.stateChanged.connect(self.toggle_servo_target_angle)
        
        servo_layout.addWidget(self.servo_angle_checkbox)
        servo_layout.addWidget(self.servo_target_angle_checkbox)
        
        # Control buttons
        buttons_group = QtWidgets.QGroupBox("Controls")
        buttons_layout = QHBoxLayout(buttons_group)
        
        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        
        self.load_btn = QPushButton("Load from File")
        self.load_btn.clicked.connect(self.load_from_file)
        
        buttons_layout.addWidget(self.clear_btn)
        buttons_layout.addWidget(self.load_btn)
        
        controls.addWidget(motor_group)
        controls.addWidget(servo_group)
        controls.addWidget(buttons_group)
        controls.addStretch()
        
        layout.addLayout(controls)
        
        # Create the plot widgets
        # Motor throttle plot
        self.motor_widget = pg.PlotWidget()
        self.motor_widget.setLabel('left', 'Throttle (%)')
        self.motor_widget.setLabel('bottom', 'Time (seconds)')
        self.motor_widget.setTitle('Motor Throttle Data')
        self.motor_widget.addLegend()
        self.motor_widget.showGrid(x=True, y=True)
        
        # Servo angle plot
        self.servo_widget = pg.PlotWidget()
        self.servo_widget.setLabel('left', 'Angle (degrees)')
        self.servo_widget.setLabel('bottom', 'Time (seconds)')
        self.servo_widget.setTitle('Servo Angle Data')
        self.servo_widget.addLegend()
        self.servo_widget.showGrid(x=True, y=True)
        
        layout.addWidget(self.motor_widget)
        layout.addWidget(self.servo_widget)
        
    def setup_data(self):
        # Data storage - using deque for efficient append/pop operations
        self.max_points = 1000  # Maximum number of points to display
        self.start_time = None
        
        # Time data (shared across all plots)
        self.time_data = deque(maxlen=self.max_points)
        
        # Motor data - store data for each motor by name
        self.motor_data = {}  # Will store {motor_name: {'throttle': deque, 'target_throttle': deque}}
        
        # Servo data - store data for each servo by name
        self.servo_data = {}  # Will store {servo_name: {'angle': deque, 'target_angle': deque}}
        
        # Plot curves - will be created dynamically as motors/servos are encountered
        self.motor_curves = {}  # Will store {motor_name: {'throttle': curve, 'target_throttle': curve}}
        self.servo_curves = {}  # Will store {servo_name: {'angle': curve, 'target_angle': curve}}
        
    def add_actuator_data(self, actuator_data):
        """Add new actuator data point"""
        current_time = datetime.now()
        
        if self.start_time is None:
            self.start_time = current_time
            
        # Calculate time relative to start
        time_seconds = (current_time - self.start_time).total_seconds()
        
        # Add time point
        self.time_data.append(time_seconds)
        
        # Process motor data
        for actuator_name, actuator_info in actuator_data.items():
            if actuator_info.get("type") == "motor":
                # Initialize motor data if first time seeing this motor
                if actuator_name not in self.motor_data:
                    self.motor_data[actuator_name] = {
                        'throttle': deque(maxlen=self.max_points),
                        'target_throttle': deque(maxlen=self.max_points)
                    }
                    # Create plot curves for this motor
                    self.motor_curves[actuator_name] = {
                        'throttle': self.motor_widget.plot(
                            pen=pg.mkPen(color='r', width=2), 
                            name=f'{actuator_name} Throttle'
                        ),
                        'target_throttle': self.motor_widget.plot(
                            pen=pg.mkPen(color='r', width=2, style=QtCore.Qt.DashLine), 
                            name=f'{actuator_name} Target Throttle'
                        )
                    }
                
                # Add data points
                throttle = actuator_info.get("throttle", 0)
                target_throttle = actuator_info.get("target_throttle", 0)
                
                self.motor_data[actuator_name]['throttle'].append(throttle)
                self.motor_data[actuator_name]['target_throttle'].append(target_throttle)
                
            elif actuator_info.get("type") == "servo":
                # Initialize servo data if first time seeing this servo
                if actuator_name not in self.servo_data:
                    self.servo_data[actuator_name] = {
                        'angle': deque(maxlen=self.max_points),
                        'target_angle': deque(maxlen=self.max_points)
                    }
                    # Create plot curves for this servo
                    self.servo_curves[actuator_name] = {
                        'angle': self.servo_widget.plot(
                            pen=pg.mkPen(color='b', width=2), 
                            name=f'{actuator_name} Angle'
                        ),
                        'target_angle': self.servo_widget.plot(
                            pen=pg.mkPen(color='b', width=2, style=QtCore.Qt.DashLine), 
                            name=f'{actuator_name} Target Angle'
                        )
                    }
                
                # Add data points
                angle = actuator_info.get("angle", 0)
                target_angle = actuator_info.get("target_angle", 0)
                
                self.servo_data[actuator_name]['angle'].append(angle)
                self.servo_data[actuator_name]['target_angle'].append(target_angle)
        
        # Update plots
        self.update_plots()
        
    def update_plots(self):
        """Update the plot curves with current data"""
        time_array = np.array(self.time_data)
        
        # Update motor plots
        for motor_name, motor_curves in self.motor_curves.items():
            if motor_name in self.motor_data:
                if self.motor_throttle_checkbox.isChecked():
                    motor_curves['throttle'].setData(
                        time_array, 
                        np.array(self.motor_data[motor_name]['throttle'])
                    )
                else:
                    motor_curves['throttle'].setData([], [])
                    
                if self.motor_target_throttle_checkbox.isChecked():
                    motor_curves['target_throttle'].setData(
                        time_array, 
                        np.array(self.motor_data[motor_name]['target_throttle'])
                    )
                else:
                    motor_curves['target_throttle'].setData([], [])
        
        # Update servo plots
        for servo_name, servo_curves in self.servo_curves.items():
            if servo_name in self.servo_data:
                if self.servo_angle_checkbox.isChecked():
                    servo_curves['angle'].setData(
                        time_array, 
                        np.array(self.servo_data[servo_name]['angle'])
                    )
                else:
                    servo_curves['angle'].setData([], [])
                    
                if self.servo_target_angle_checkbox.isChecked():
                    servo_curves['target_angle'].setData(
                        time_array, 
                        np.array(self.servo_data[servo_name]['target_angle'])
                    )
                else:
                    servo_curves['target_angle'].setData([], [])
    
    def toggle_motor_throttle(self):
        """Toggle motor throttle visibility"""
        self.update_plots()
        
    def toggle_motor_target_throttle(self):
        """Toggle motor target throttle visibility"""
        self.update_plots()
        
    def toggle_servo_angle(self):
        """Toggle servo angle visibility"""
        self.update_plots()
        
    def toggle_servo_target_angle(self):
        """Toggle servo target angle visibility"""
        self.update_plots()
    
    def clear_data(self):
        """Clear all data and reset plots"""
        self.time_data.clear()
        
        # Clear motor data
        for motor_name in self.motor_data:
            self.motor_data[motor_name]['throttle'].clear()
            self.motor_data[motor_name]['target_throttle'].clear()
            
        # Clear servo data
        for servo_name in self.servo_data:
            self.servo_data[servo_name]['angle'].clear()
            self.servo_data[servo_name]['target_angle'].clear()
            
        # Clear plots
        for motor_curves in self.motor_curves.values():
            motor_curves['throttle'].setData([], [])
            motor_curves['target_throttle'].setData([], [])
            
        for servo_curves in self.servo_curves.values():
            servo_curves['angle'].setData([], [])
            servo_curves['target_angle'].setData([], [])
            
        self.start_time = None
        
    def load_from_file(self):
        """Load actuator data from a log file"""
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Actuator Data", "", "Log Files (*.log);;All Files (*)"
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as file:
                    # Clear existing data first
                    self.clear_data()
                    
                    for line_num, line in enumerate(file, 1):
                        try:
                            data = json.loads(line.strip())
                            if data.get("type") == "actuators":
                                actuator_data = data.get("data", {})
                                self.add_actuator_data(actuator_data)
                        except json.JSONDecodeError:
                            print(f"Warning: Skipping invalid JSON on line {line_num}")
                            continue
                            
            except Exception as e:
                QtWidgets.QMessageBox.critical(
                    self, "Error", f"Failed to load file: {str(e)}"
                )


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
        """Load attitude and IMU data from a selected flight_data.log file"""
        try:
            # Open file dialog to select flight_data.log file
            logs_dir = "./GUI/Logs"
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, 
                "Select Flight Data Log File", 
                logs_dir,
                "Log Files (*.log);;All Files (*)"
            )
            
            if not file_path:
                return  # User cancelled
                
            if not os.path.exists(file_path):
                QtWidgets.QMessageBox.warning(self, "Warning", "Selected file does not exist!")
                return
                
            self.clear_data()
            
            with open(file_path, 'r', encoding='utf-8') as f:
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
                        time_seconds = i / 100  # Assume 100Hz data rate
                        
                        # Extract IMU values
                        ax = imu_data.get("ax", 0)
                        ay = imu_data.get("ay", 0)
                        az = imu_data.get("az", 0)
                        gx = imu_data.get("gx", 0)
                        gy = imu_data.get("gy", 0)
                        gz = imu_data.get("gz", 0)
                        
                        self.time_data.append(time_seconds)
                        self.ax_data.append(ax / 1000.0)
                        self.ay_data.append(ay / 1000.0)
                        self.az_data.append(az / 1000.0)
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

def main():
    app = QApplication(sys.argv)
    viewer = FlightViewer()
    viewer.show()
    # Ensure log files are closed on app exit
    app.aboutToQuit.connect(viewer.close)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
