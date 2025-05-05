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

import os
import csv

csv_filename = "motor_measurement.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
# Schreibe die Kopfzeile
csv_writer.writerow(["time", "u", "y1", "y2"])


port = "5556"
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.101:%s" % port)

app = QApplication(sys.argv)
running = True

plot_widget = LivePlotWidget(title="Line Plot @ 50Hz")
plot_widget.plotItem.showGrid(x=True, y=True)
plot_widget.plotItem.addLegend()

plot_curve1 = LiveLinePlot(pen="r", name="u")  # Rot
plot_widget.addItem(plot_curve1)
data_connector1 = DataConnector(plot_curve1, max_points=600, update_rate=100)

plot_curve2 = LiveLinePlot(pen="g", name = "y1")  # Blau
plot_widget.addItem(plot_curve2)
data_connector2 = DataConnector(plot_curve2, max_points=600, update_rate=100)

plot_curve3 = LiveLinePlot(pen="b", name = "y2")  # Blau
plot_widget.addItem(plot_curve3)
data_connector3 = DataConnector(plot_curve3, max_points=600, update_rate=100)

start_time = time.perf_counter()

def update(connector1, connector2, connector3, connector4, connector5, connector6, connector7, connector8):
    while running:
        socket.send_string("blabla")
        data = json.loads(socket.recv_string())
        current_time = time.perf_counter() - start_time
        value1 = data["motor_left"]["sp"]
        value2 = data["motor_left"]["velocity"]
        value3 = data["motor_right"]["velocity"]
        connector1.cb_append_data_point(value1, current_time)
        connector2.cb_append_data_point(value2, current_time)
        connector3.cb_append_data_point(value3, current_time)
        csv_writer.writerow([current_time, value1, value2, value3])

        sleep(0.001)


plot_widget.showMaximized()
Thread(target=update, args=(data_connector1, data_connector2, data_connector3)).start()
app.exec()
running = False
csv_file.close()
