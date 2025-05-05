import sys
import time
from math import sin
from threading import Thread
from time import sleep

from PyQt6.QtWidgets import QApplication

from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget

import zmq
import json

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)

app = QApplication(sys.argv)
running = True

plot_widget = LivePlotWidget(title="Line Plot @ 50Hz")
plot_widget.plotItem.showGrid(x=True, y=True)
plot_widget.plotItem.addLegend()

plot_curve1 = LiveLinePlot(pen="r", name="Pitch")  # Rot
plot_widget.addItem(plot_curve1)
data_connector1 = DataConnector(plot_curve1, max_points=600, update_rate=100)

plot_curve2 = LiveLinePlot(pen="g", name = "Pitch_d")  # Blau
plot_widget.addItem(plot_curve2)
data_connector2 = DataConnector(plot_curve2, max_points=600, update_rate=100)

plot_curve3 = LiveLinePlot(pen="b", name = "Yaw")  # Blau
plot_widget.addItem(plot_curve3)
data_connector3 = DataConnector(plot_curve3, max_points=600, update_rate=100)

plot_curve4 = LiveLinePlot(pen="m", name = "Left_Pos")  # Blau
plot_widget.addItem(plot_curve4)
data_connector4 = DataConnector(plot_curve4, max_points=600, update_rate=100)

plot_curve5 = LiveLinePlot(pen="c", name = "Left_Vel")  # Blau
plot_widget.addItem(plot_curve5)
data_connector5 = DataConnector(plot_curve5, max_points=600, update_rate=100)

plot_curve6 = LiveLinePlot(pen="y", name = "Right_Pos")  # Blau
plot_widget.addItem(plot_curve6)
data_connector6 = DataConnector(plot_curve6, max_points=600, update_rate=100)

plot_curve7 = LiveLinePlot(pen=(255,165,0), name = "Right_Vel")  # Blau
plot_widget.addItem(plot_curve7)
data_connector7 = DataConnector(plot_curve7, max_points=600, update_rate=100)

plot_curve8 = LiveLinePlot(pen=(128,0,128), name = "PID_out")  # Blau
plot_widget.addItem(plot_curve8)
data_connector8 = DataConnector(plot_curve8, max_points=600, update_rate=100)

start_time = time.perf_counter()

def update(connector1, connector2, connector3, connector4, connector5, connector6, connector7, connector8):
    while running:
        socket.send_string("blabla")
        data = json.loads(socket.recv_string())
        current_time = time.perf_counter() - start_time
        value1 = data["imu"]["pitch_LP"]
        value2 = data["imu"]["gyro_x_LP"]
        value3 = data["imu"]["yaw"]
        value4 = data["motor_left"]["position"]
        value5 = data["motor_left"]["velocity_LP"]
        value6 = data["motor_right"]["position"]
        value7 = data["motor_right"]["velocity_LP"]
        value8 = data["lqr"]["out"]
        connector1.cb_append_data_point(value1, current_time)
        connector2.cb_append_data_point(value2, current_time)
        connector3.cb_append_data_point(value3, current_time)
        connector4.cb_append_data_point(value4, current_time)
        connector5.cb_append_data_point(value5, current_time)
        connector6.cb_append_data_point(value6, current_time)
        connector7.cb_append_data_point(value7, current_time)
        connector8.cb_append_data_point(value8, current_time)
        sleep(0.001)


plot_widget.showMaximized()
Thread(target=update, args=(data_connector1, data_connector2, data_connector3, data_connector4, data_connector5, data_connector6, data_connector7, data_connector8)).start()
app.exec()
running = False
