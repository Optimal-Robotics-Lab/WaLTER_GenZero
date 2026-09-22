import time
import zmq

""" Test Script for sending fake motor data to a ZeroMQ socket for plotting."""

class DataSender():
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        self.log_interval = 0.05
        self.last_log_time = 0

    def send_data(self, sim_time, data):
        if(sim_time-self.last_log_time>=self.log_interval):
            self.data_to_send = data
            self.socket.send_pyobj(self.data_to_send)
            self.last_log_time = sim_time
