# import sys
# import pyqtgraph as pg
# from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout
# import zmq
# import numpy as np
# import time


# class SpeedTorquePlotter(QWidget):
#     def __init__(self, num_motors, **kwargs):
#         super().__init__()
#         # Set up ZeroMQ context and socket
#         self.context = zmq.Context()
#         self.socket = self.context.socket(zmq.SUB)
#         self.socket.connect("tcp://localhost:5555")
#         self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
#         self.timer = pg.QtCore.QTimer()
#         self.timer.timeout.connect(self.update_plot)
#         self.timer.start(0)
#         self.num_motors = num_motors
        
#         # Store historical data
#         self.historical_data = [{'time': [], 'torque': [], 'speed': []} for _ in range(self.num_motors)]
#         # Maximum number of points to keep (to prevent memory issues)
#         self.max_points = 500  # Adjust this based on your needs

#         if 'names' in kwargs:
#             self.names = kwargs['names']
#         else:
#             self.names = [f'Motor {i+1}' for i in range(num_motors)]
        
#         # Store speed-torque reference lines
#         self.speed_torque_lines = [None] * self.num_motors
        
#         self.initUI()

#     def initUI(self):
#         self.setWindowTitle("PyQt Scatter Plot with History")
#         self.setGeometry(100, 100, 400, 1000)

#         # Initializing plot widgets and plots
#         self.plot_widgets = []
#         self.plots = []
#         layout = QVBoxLayout(self)

#         # Initialize the plot widgets and plots
#         for i in range(self.num_motors):
#             plot_widget = pg.PlotWidget()
#             plot_widget.setLabel('left', f'{self.names[i]} Torque (Nm)')
#             plot_widget.setLabel('bottom', 'Speed (rpm)')
            
#             # Use PlotDataItem instead of ScatterPlotItem for better performance with many points
#             plot = plot_widget.plot(pen=None, symbol='o', symbolSize=5, symbolBrush='r')
#             self.plot_widgets.append(plot_widget)
#             self.plots.append(plot)
#             layout.addWidget(plot_widget)
        
#         self.setLayout(layout)
#         self.show()

#     def add_speed_torque_line(self, stall_torque, no_load_speed, motor_index, gear_ratio=1):
#         """
#         Add a speed-torque reference line for a specific motor.
        
#         Parameters:
#         - stall_torque: Torque at zero speed (y-intercept)
#         - no_load_speed: Speed at zero torque (x-intercept)
#         - motor_index: Index of the motor (0-based)
#         """
#         if motor_index < 0 or motor_index >= self.num_motors:
#             print(f"Error: Motor index {motor_index} out of range (0-{self.num_motors-1})")
#             return
        
#         # Create speed-torque line data points
#         stall_torque = stall_torque * gear_ratio
#         no_load_speed = no_load_speed / gear_ratio

#         x_data = [0, no_load_speed]
#         y_data = [stall_torque, 0]
        
#         # Add or update the speed-torque line
#         if self.speed_torque_lines[motor_index] is None:
#             # Create a new line with blue color and dashed style
#             self.speed_torque_lines[motor_index] = self.plot_widgets[motor_index].plot(
#                 x_data, y_data, 
#                 pen=pg.mkPen(color='b', width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
#                 name="Speed-Torque Curve"
#             )
#         else:
#             # Update existing line
#             self.speed_torque_lines[motor_index].setData(x_data, y_data)
        
#         # Add a legend if not already present
#         if not hasattr(self.plot_widgets[motor_index], 'legend') or self.plot_widgets[motor_index].legend is None:
#             self.plot_widgets[motor_index].addLegend()
        
#         # Update axis limits to ensure the line is visible
#         self.plot_widgets[motor_index].setXRange(0, no_load_speed * 1.1)
#         self.plot_widgets[motor_index].setYRange(0, stall_torque * 1.1)
        
#         print(f"Added speed-torque line for motor {motor_index}: "
#               f"Stall Torque = {stall_torque} Nm, No Load Speed = {no_load_speed} rpm")

#     def update_plot(self):
#         # If no data is received, return
#         if self.socket.poll(0) == 0:
#             return

#         data_dict = self.get_data()
#         t = data_dict['time']
        
#         for i in range(self.num_motors):
#             motor_key = f'motor{i+1}_torque'
#             motor_speed_key = f'motor{i+1}_speed'
#             if motor_key in data_dict:
#                 motor_torque = data_dict[motor_key]

#                 if motor_speed_key in data_dict:
#                     motor_speed = data_dict[motor_speed_key]
                
#                 # Append new data to historical data
#                 self.historical_data[i]['time'].append(t)
#                 self.historical_data[i]['speed'].append(motor_speed)
#                 self.historical_data[i]['torque'].append(motor_torque)
                
#                 # Limit the number of points to prevent memory issues
#                 if len(self.historical_data[i]['speed']) > self.max_points:
#                     self.historical_data[i]['speed'] = self.historical_data[i]['speed'][-self.max_points:]
#                     self.historical_data[i]['torque'] = self.historical_data[i]['torque'][-self.max_points:]
                
#                 # Update the plot with all historical data
#                 self.plots[i].setData(
#                     x=self.historical_data[i]['speed'], 
#                     y=self.historical_data[i]['torque']
#                 )

#     def get_data(self):
#         data = self.socket.recv_pyobj()
#         return data

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     # genplot = GeneralPlotter(num_plots=1)
#     scatter_app = SpeedTorquePlotter(num_motors=3, names = ['BR Hip', 'BR Knee', 'BR Wheel'])
    

#     # STC lines for Kollmorgen frameless actuators
#     # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=0, gear_ratio=160)  # BR Hip
#     # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=1, gear_ratio=120)  # BR Knee
#     # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=2, gear_ratio=30)  # BR Wheel
    
#     # STC lines for RMDX actuators
#     scatter_app.add_speed_torque_line(stall_torque=16, no_load_speed=2500, motor_index=0, gear_ratio=20)  # BR Hip
#     scatter_app.add_speed_torque_line(stall_torque=16, no_load_speed=2500, motor_index=1, gear_ratio=20)  # BR Knee
#     scatter_app.add_speed_torque_line(stall_torque=5.5, no_load_speed=3150, motor_index=2, gear_ratio=20)  # BR Wheel
    

#     sys.exit(app.exec())

import sys
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout
import zmq
import numpy as np
import time


class SpeedTorquePlotter(QWidget):
    def __init__(self, num_motors, **kwargs):
        super().__init__()
        # Set up ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(0)
        self.num_motors = num_motors
        
        # Store historical data
        self.historical_data = [{'time': [], 'torque': [], 'speed': []} for _ in range(self.num_motors)]
        # Maximum number of points to keep (to prevent memory issues)
        self.max_points = 500  # Adjusted to 500 as in your code

        if 'names' in kwargs:
            self.names = kwargs['names']
        else:
            self.names = [f'Motor {i+1}' for i in range(num_motors)]
        
        # Store speed-torque reference lines
        self.speed_torque_lines = [None] * self.num_motors
        
        # Store continuous operation limits
        self.continuous_torque_lines = [None] * self.num_motors
        self.continuous_speed_lines = [None] * self.num_motors
        self.continuous_regions = [None] * self.num_motors
        
        self.initUI()

    def initUI(self):
        self.setWindowTitle("PyQt Scatter Plot with History")
        self.setGeometry(15000, 100, 400, 1000)

        # Initializing plot widgets and plots
        self.plot_widgets = []
        self.plots = []
        layout = QVBoxLayout(self)

        # Initialize the plot widgets and plots
        for i in range(self.num_motors):
            plot_widget = pg.PlotWidget()
            plot_widget.setTitle(f'{self.names[i]} Speed-Torque Curve')
            plot_widget.setLabel('left', 'Torque (Nm)')
            plot_widget.setLabel('bottom', 'Speed (rpm)')
            
            # Use PlotDataItem instead of ScatterPlotItem for better performance with many points
            plot = plot_widget.plot(pen=None, symbol='o', symbolSize=5, symbolBrush='r')
            self.plot_widgets.append(plot_widget)
            self.plots.append(plot)
            layout.addWidget(plot_widget)
        
        self.setLayout(layout)
        self.show()

    def add_speed_torque_line(self, stall_torque, no_load_speed, motor_index, gear_ratio=1):
        """
        Add a speed-torque reference line for a specific motor.
        
        Parameters:
        - stall_torque: Torque at zero speed (y-intercept)
        - no_load_speed: Speed at zero torque (x-intercept)
        - motor_index: Index of the motor (0-based)
        - gear_ratio: Gear ratio for torque multiplication and speed reduction
        """
        if motor_index < 0 or motor_index >= self.num_motors:
            print(f"Error: Motor index {motor_index} out of range (0-{self.num_motors-1})")
            return
        
        # Create speed-torque line data points
        stall_torque = stall_torque * gear_ratio
        no_load_speed = no_load_speed / gear_ratio

        x_data = [0, no_load_speed]
        y_data = [stall_torque, 0]
        
        # Add or update the speed-torque line
        if self.speed_torque_lines[motor_index] is None:
            # Create a new line with blue color and dashed style
            self.speed_torque_lines[motor_index] = self.plot_widgets[motor_index].plot(
                x_data, y_data, 
                pen=pg.mkPen(color='b', width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
                name="Speed-Torque Curve"
            )
        else:
            # Update existing line
            self.speed_torque_lines[motor_index].setData(x_data, y_data)
        
        # Add a legend if not already present
        if not hasattr(self.plot_widgets[motor_index], 'legend') or self.plot_widgets[motor_index].legend is None:
            self.plot_widgets[motor_index].addLegend()
        
        # Update axis limits to ensure the line is visible
        self.plot_widgets[motor_index].setXRange(0, no_load_speed * 1.1)
        self.plot_widgets[motor_index].setYRange(0, stall_torque * 1.1)
        
        print(f"Added speed-torque line for motor {motor_index}: "
              f"Stall Torque = {stall_torque} Nm, No Load Speed = {no_load_speed} rpm")

    def add_continuous_operation_region(self, continuous_torque, continuous_speed, motor_index, gear_ratio=1):
        """
        Add continuous operation region defined by continuous torque and speed limits.
        
        Parameters:
        - continuous_torque: Maximum continuous torque rating
        - continuous_speed: Maximum continuous speed rating
        - motor_index: Index of the motor (0-based)
        - gear_ratio: Gear ratio for torque multiplication and speed reduction
        """
        if motor_index < 0 or motor_index >= self.num_motors:
            print(f"Error: Motor index {motor_index} out of range (0-{self.num_motors-1})")
            return
            
        # Apply gear ratio to the continuous values
        continuous_torque = continuous_torque * gear_ratio
        continuous_speed = continuous_speed / gear_ratio
            
        # Get the plot widget for this motor
        plot_widget = self.plot_widgets[motor_index]
        
        # Get current axes limits to determine where to draw the lines
        x_range = plot_widget.getViewBox().viewRange()[0]
        y_range = plot_widget.getViewBox().viewRange()[1]
        max_x = x_range[1]
        max_y = y_range[1]
        
        # Create continuous torque line (horizontal line)
        if self.continuous_torque_lines[motor_index] is None:
            self.continuous_torque_lines[motor_index] = plot_widget.plot(
                [0, continuous_speed], 
                [continuous_torque, continuous_torque],
                pen=pg.mkPen(color='g', width=2),
            )
        else:
            self.continuous_torque_lines[motor_index].setData(
                [0, continuous_speed], 
                [continuous_torque, continuous_torque]
            )
            
        # Create continuous speed line (vertical line)
        if self.continuous_speed_lines[motor_index] is None:
            self.continuous_speed_lines[motor_index] = plot_widget.plot(
                [continuous_speed, continuous_speed], 
                [0, continuous_torque],
                pen=pg.mkPen(color='g', width=2),
            )
        else:
            self.continuous_speed_lines[motor_index].setData(
                [continuous_speed, continuous_speed], 
                [0, continuous_torque]
            )
            
        # Create a semi-transparent fill for the continuous operation region
        # First, create the polygon points (counterclockwise)
        points = np.array([
            [0, 0],                        # Origin
            [0, continuous_torque],        # Up the torque axis
            [continuous_speed, continuous_torque],  # Across to speed limit
            [continuous_speed, 0],         # Down to speed axis
            [0, 0]                         # Back to origin
        ])
            
        # Create or update the region fill
        if self.continuous_regions[motor_index] is None:
            # Create a QGraphicsPathItem for the filled region
            path = pg.arrayToQPath(points[:, 0], points[:, 1])
            region = pg.QtWidgets.QGraphicsPathItem(path)
            region.setBrush(pg.mkBrush(color=pg.mkColor(0, 255, 0, 50)))  # Semi-transparent green
            region.setPen(pg.mkPen(None))  # No border
            plot_widget.addItem(region)
            self.continuous_regions[motor_index] = region
        else:
            # Update existing region
            path = pg.arrayToQPath(points[:, 0], points[:, 1])
            self.continuous_regions[motor_index].setPath(path)
            
        print(f"Added continuous operation region for motor {motor_index}: "
              f"Continuous Torque = {continuous_torque} Nm, Continuous Speed = {continuous_speed} rpm")

    def update_plot(self):
        # If no data is received, return
        if self.socket.poll(0) == 0:
            return

        data_dict = self.get_data()
        t = data_dict['time']
        
        for i in range(self.num_motors):
            motor_key = f'motor{i+1}_torque'
            motor_speed_key = f'motor{i+1}_speed'
            if motor_key in data_dict:
                motor_torque = data_dict[motor_key]

                if motor_speed_key in data_dict:
                    motor_speed = data_dict[motor_speed_key]
                
                # Append new data to historical data
                self.historical_data[i]['time'].append(t)
                self.historical_data[i]['speed'].append(motor_speed)
                self.historical_data[i]['torque'].append(motor_torque)
                
                # Limit the number of points to prevent memory issues
                if len(self.historical_data[i]['speed']) > self.max_points:
                    self.historical_data[i]['speed'] = self.historical_data[i]['speed'][-self.max_points:]
                    self.historical_data[i]['torque'] = self.historical_data[i]['torque'][-self.max_points:]
                
                # Update the plot with all historical data
                self.plots[i].setData(
                    x=self.historical_data[i]['speed'], 
                    y=self.historical_data[i]['torque']
                )

    def get_data(self):
        data = self.socket.recv_pyobj()
        return data

if __name__ == '__main__':

    # Initialize the application
    app = QApplication(sys.argv)
    scatter_app = SpeedTorquePlotter(num_motors=3, names = ['BR Hip', 'BR Knee', 'BR Wheel'])
    
    # # Speed torque curves and continuous operation region for RMD actuators
    # scatter_app.add_speed_torque_line(stall_torque=16, no_load_speed=2500, motor_index=0, gear_ratio=20)  # BR Hip
    # scatter_app.add_speed_torque_line(stall_torque=16, no_load_speed=2500, motor_index=1, gear_ratio=20)  # BR Knee
    # # scatter_app.add_speed_torque_line(stall_torque=5.5, no_load_speed=3150, motor_index=2, gear_ratio=20)  # BR Wheel
    
    # scatter_app.add_continuous_operation_region(continuous_torque=5, continuous_speed=1500, motor_index=0, gear_ratio=20)  # BR Hip
    # scatter_app.add_continuous_operation_region(continuous_torque=5, continuous_speed=1500, motor_index=1, gear_ratio=20)  # BR Knee
    # # scatter_app.add_continuous_operation_region(continuous_torque=2.15, continuous_speed=2000, motor_index=2, gear_ratio=20)  # BR Wheel
    



    # # Speed torque curves and continuous operation region for frameless kollmorgen actuators
    # # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=0, gear_ratio=160)  # BR Hip
    # # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=1, gear_ratio=120)  # BR Knee
    # # scatter_app.add_continuous_operation_region(continuous_torque=0.675, continuous_speed=3500, motor_index=0, gear_ratio=160)  # BR Hip
    # # scatter_app.add_continuous_operation_region(continuous_torque=0.9, continuous_speed=3500, motor_index=1, gear_ratio=120)  # BR Knee

    # # Frameless wheel option 1 (Old)
    # # scatter_app.add_speed_torque_line(stall_torque=1.8, no_load_speed=8000, motor_index=2, gear_ratio=30)  # BR Wheel
    # # scatter_app.add_continuous_operation_region(continuous_torque=1.2, continuous_speed=3500, motor_index=2, gear_ratio=30)  # BR Wheel

    # # Frameless wheel option 2 (new)
    # scatter_app.add_speed_torque_line(stall_torque=14.5, no_load_speed=3100, motor_index=2, gear_ratio=8)  # BR Wheel
    # scatter_app.add_continuous_operation_region(continuous_torque=4.41, continuous_speed=3000, motor_index=2, gear_ratio=8)  # BR Wheel    

    # Add speed torque curves for WaLTER Sr Actuators:
    scatter_app.add_speed_torque_line(stall_torque=2.2, no_load_speed=5000, motor_index=0, gear_ratio=6)  # BR Hip
    scatter_app.add_speed_torque_line(stall_torque=2.2, no_load_speed=5000, motor_index=1, gear_ratio=9.5)  # BR Knee
    scatter_app.add_speed_torque_line(stall_torque=4.9, no_load_speed=5000, motor_index=2, gear_ratio=1)  # BR Wheel

    # Add continuous operation regions for WaLTER Sr Actuators:
    scatter_app.add_continuous_operation_region(continuous_torque=0.7, continuous_speed=3000, motor_index=0, gear_ratio=6)  # BR Hip
    scatter_app.add_continuous_operation_region(continuous_torque=0.7, continuous_speed=3000, motor_index=1, gear_ratio=9.5)  # BR Knee
    scatter_app.add_continuous_operation_region(continuous_torque=1.47, continuous_speed=3000, motor_index=2, gear_ratio=1)  # BR Wheel

    sys.exit(app.exec())