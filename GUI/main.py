import sys
import json
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtSerialPort
from PyQt5.QtWidgets import QApplication
from PyQt5.QtSerialPort import QSerialPort
from pyqtgraph.opengl import GLViewWidget, GLBoxItem, GLGridItem, GLLinePlotItem, GLMeshItem, MeshData
from pyqtgraph import Vector
import trimesh

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
    return mesh_item

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
        self.resize(1000, 600)

        # 3D view setup
        self.view = GLViewWidget()
        self.view.setMinimumHeight(300)
        self.view.opts['distance'] = 5
        # self.cube = GLBoxItem(size=Vector(1, 0.2, 0.5), color=(255, 0, 0, 255))
        # self.view.addItem(self.cube)

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

        self.airplane = load_mesh("./GUI/data/mesh/plane.stl")
        self.view.addItem(self.airplane)
        # Debugging airplane mesh rotation
        # self.rotation_angle = 1
        # self.rotation_timer = QtCore.QTimer()
        # self.rotation_timer.timeout.connect(self.rotate_airplane)
        # self.rotation_timer.start(100)

        # Serial GUI controls
        self.message_le = QtWidgets.QLineEdit()
        self.send_btn = QtWidgets.QPushButton("Send", clicked=self.send)
        self.output_te = QtWidgets.QTextEdit(readOnly=True)
        self.button = QtWidgets.QPushButton("Connect", checkable=True, toggled=self.on_toggled)

        # Layout
        layout = QtWidgets.QVBoxLayout(self)
        top_controls = QtWidgets.QHBoxLayout()
        top_controls.addWidget(self.message_le)
        top_controls.addWidget(self.send_btn)
        layout.addLayout(top_controls)
        layout.addWidget(self.output_te)
        layout.addWidget(self.view)
        layout.addWidget(self.button)

        # Serial port setup
        self.serial = QSerialPort(
            "COM4",
            baudRate=QtSerialPort.QSerialPort.Baud115200,
            readyRead=self.receive
        )
        self.buffer = b""

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

    @QtCore.pyqtSlot()
    def receive(self):
        self.buffer += self.serial.readAll().data() # .readAll()
        while START_DELIM in self.buffer and END_DELIM in self.buffer:
            start = self.buffer.find(START_DELIM)
            end = self.buffer.find(END_DELIM, start)
            if end == -1:
                break
            message = self.buffer[start + 1:end]
            self.buffer = self.buffer[end + 1:]
            try:
                data = json.loads(message)
                if data.get("type") == "imu":
                    self.update_orientation(data.get("orientation", {}))
                elif data.get("type") == "debug":
                    self.output_te.append(f"{data.get('msg')}")
            except json.JSONDecodeError:
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

        # self.airplane.resetTransform()
        self.airplane.rotate(yaw_deg, 0, 0, 1)    # Yaw (Z axis)
        self.airplane.rotate(pitch_deg, 0, 1, 0)  # Pitch (Y axis)
        self.airplane.rotate(roll_deg, 1, 0, 0)   # Roll (X axis)

def main():
    app = QApplication(sys.argv)
    viewer = IMUViewer()
    viewer.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
