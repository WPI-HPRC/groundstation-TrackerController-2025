import sys
import time
import serial
import threading
import numpy as np
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QHBoxLayout,
                             QGridLayout, QCheckBox)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

class TuningDashboard(QWidget):
    def __init__(self):
        super().__init__()

        self.serial = serial.Serial('COM4', 115200, timeout=0.1)  # Adjust COM port as needed
        time.sleep(2)

        self.last_pos = None
        self.last_vel = None
        self.last_acc = None
        self.last_time = time.time()

        self.initUI()
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.read_serial)
        self.data_timer.start(50)

    def initUI(self):
        self.setWindowTitle("Stepper Tuning Dashboard")
        layout = QVBoxLayout()

        self.graph = pg.PlotWidget(title="Position, Velocity, Acceleration")
        self.graph.addLegend()
        
        # Position Plot
        self.pos_line = self.graph.plot(pen='r', name="Desired Position (°)")
        self.cur_pos_line = self.graph.plot(pen='g', name="Actual Position (°)")

        # Velocity Plot
        self.vel_line = self.graph.plot(pen='b', name="Desired Velocity (°/s)")
        self.cur_vel_line = self.graph.plot(pen='c', name="Actual Velocity (°/s)")

        # Acceleration Plot
        self.acc_line = self.graph.plot(pen='y', name="Desired Acceleration (°/s²)")
        self.cur_acc_line = self.graph.plot(pen='m', name="Actual Acceleration (°/s²)")

        layout.addWidget(self.graph)

        self.pos_data, self.cur_pos_data, self.vel_data, self.cur_vel_data, self.acc_data, self.cur_acc_data = [], [], [], [], [], []
        self.times = []

        grid = QGridLayout()
        labels = [
            ("Max Velocity (°/s)", 'max_vel'),
            ("Max Acceleration (°/s²)", 'max_acc'),
            ("Max Jerk (°/s³)", 'max_jerk'),
            ("kP", 'kp'),
            ("kD", 'kd'),
            ("Gravity FF", 'grav_ff'),
            ("Acceptable Error (°)", 'acc_err'),
            ("Target Position (°)", 'target_pos')
        ]

        self.inputs = {}
        for i, (label, key) in enumerate(labels):
            grid.addWidget(QLabel(label), i, 0)
            self.inputs[key] = QLineEdit()
            grid.addWidget(self.inputs[key], i, 1)

        layout.addLayout(grid)

        self.brake_toggle = QCheckBox("Brake Mode")
        layout.addWidget(self.brake_toggle)

        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self.apply_settings)
        layout.addWidget(apply_btn)

        self.summary_label = QLabel("Tuning Parameters Summary")
        layout.addWidget(self.summary_label)

        self.setLayout(layout)

    def apply_settings(self):
        values = {key: self.inputs[key].text() for key in self.inputs}
        try:
            formatted = f"APPLY," + ",".join([str(float(v)) for v in values.values()]) + f",{int(self.brake_toggle.isChecked())}\n"
            self.serial.write(formatted.encode())
            self.summary_label.setText("Current Params: " + formatted)
        except ValueError:
            self.summary_label.setText("Error: One or more invalid entries")

    def read_serial(self):
        while self.serial.in_waiting:
            try:
                line = self.serial.readline().decode().strip()
                if not line: continue
                parts = line.split(',')
                if len(parts) < 2: continue

                # Parse desired and actual values from the serial data
                des_pos = float(parts[0])
                act_pos = float(parts[1])
                des_vel = float(parts[2])
                act_vel = float(parts[3])
                des_acc = float(parts[4])
                act_acc = float(parts[5])

                # Get the time difference and calculate velocity and acceleration
                now = time.time()
                dt = now - self.last_time if self.last_time else 1e-3

                if self.last_pos is not None:
                    vel = (act_pos - self.last_pos) / dt
                    acc = (vel - self.last_vel) / dt
                else:
                    vel = 0
                    acc = 0

                # Update stored values for the next iteration
                self.last_time = now
                self.last_pos = act_pos
                self.last_vel = vel
                self.last_acc = acc

                # Append new data to respective lists
                self.times.append(now)
                self.pos_data.append(des_pos)
                self.cur_pos_data.append(act_pos)
                self.vel_data.append(des_vel)
                self.cur_vel_data.append(act_vel)
                self.acc_data.append(des_acc)
                self.cur_acc_data.append(act_acc)

                self.trim_data()
                self.update_plot()

            except Exception as e:
                print(f"Parse error: {e}")

    def trim_data(self, max_length=500):
        self.times = self.times[-max_length:]
        self.pos_data = self.pos_data[-max_length:]
        self.cur_pos_data = self.cur_pos_data[-max_length:]
        self.vel_data = self.vel_data[-max_length:]
        self.cur_vel_data = self.cur_vel_data[-max_length:]
        self.acc_data = self.acc_data[-max_length:]
        self.cur_acc_data = self.cur_acc_data[-max_length:]

    def update_plot(self):
        # Update plot for all data
        self.pos_line.setData(self.times, self.pos_data)
        self.cur_pos_line.setData(self.times, self.cur_pos_data)
        self.vel_line.setData(self.times, self.vel_data)
        self.cur_vel_line.setData(self.times, self.cur_vel_data)
        self.acc_line.setData(self.times, self.acc_data)
        self.cur_acc_line.setData(self.times, self.cur_acc_data)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TuningDashboard()
    window.show()
    sys.exit(app.exec_())
