#!/usr/bin/env python
import rospy

import sys
import matplotlib
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QRadioButton, QGroupBox, QHBoxLayout, QMessageBox, QFileDialog, QComboBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d import Axes3D

from controller_manager_msgs.srv import ListControllers

import moveit_commander
from move_test import Simulation
from tf_tcp_base_publisher import TF_TCP_Base
from dtw_plot_3d import dtw_plot_3d, plot_dtw_3d_test
from filter import get_move_group_feedback_topics

class TrajectoryDTWApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Trajectory DTW')
        self.setGeometry(100, 100, 1400, 600)  # (x, y, width, height)
        self.init_3d_plot()
        self.init_dtw_plot()
        self.initUI()
        self.user_choices = UserChoices()  # Initialize user choices to None

    def initUI(self):
        # Create a label
        label = QLabel('Select from the options below', self)

        # Create a group box for the radio buttons to choose the trajectory type
        trajectory_group_box = QGroupBox('Choose the trajectory type', self)

        # Create radio buttons for trajectory type within the group box
        self.trajectory_carthesian = QRadioButton('Carthesian Path', trajectory_group_box)
        self.trajectory_joint = QRadioButton('Joint Path', trajectory_group_box)

        # Create a group box for the radio buttons to choose the shape
        shape_group_box = QGroupBox('Choose the shape of the trajectory', self)
        
        # Create a group box for the topic list
        topic_group_box = QGroupBox('Choose topics to listen from', self)
        topic = QComboBox(topic_group_box)
        topic.addItems(self.get_topics())

        # Create radio buttons for the shape options within the group box
        self.shape_square = QRadioButton('Square', shape_group_box)
        self.shape_circle = QRadioButton('Circle', shape_group_box)
        self.shape_line = QRadioButton('Line', shape_group_box)

        # Create a button for running the task
        run_button = QPushButton('Run DTW', self)

        # Create a button for saving the plots to PDF
        save_button = QPushButton('Save to PDF', self)

        # Create a layout for the topic selection group box
        topic_group_box_layout = QVBoxLayout()
        topic_group_box_layout.addWidget(topic)
        topic_group_box.setLayout(topic_group_box_layout)

        # Create a layout for the shape group box
        shape_group_box_layout = QVBoxLayout()
        shape_group_box_layout.addWidget(self.shape_square)
        shape_group_box_layout.addWidget(self.shape_circle)
        shape_group_box_layout.addWidget(self.shape_line)
        shape_group_box.setLayout(shape_group_box_layout)

        # Create a layout for the trajectory group box
        trajectory_group_box_layout = QVBoxLayout()
        trajectory_group_box_layout.addWidget(self.trajectory_carthesian)
        trajectory_group_box_layout.addWidget(self.trajectory_joint)
        trajectory_group_box.setLayout(trajectory_group_box_layout)

        # Create a layout and add the label, the group boxes, and the button
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(topic_group_box)
        layout.addWidget(trajectory_group_box)
        layout.addWidget(shape_group_box)

        # Create a horizontal layout for the plot, control buttons, and DTW plot
        plot_layout = QHBoxLayout()
        plot_layout.addLayout(layout)
        plot_layout.addWidget(self.canvas)
        layout.addWidget(run_button)
        layout.addWidget(save_button)
        plot_layout.addWidget(self.dtw_canvas)

        # Set the layout for the window
        self.setLayout(plot_layout)

        # Connect radio button signals to update user_choices
        self.trajectory_carthesian.toggled.connect(self.update_user_choices)
        self.trajectory_joint.toggled.connect(self.update_user_choices)
        self.shape_square.toggled.connect(self.update_user_choices)
        self.shape_circle.toggled.connect(self.update_user_choices)
        self.shape_line.toggled.connect(self.update_user_choices)

        # Connect the button click events to the functions
        run_button.clicked.connect(self.run_program)
        save_button.clicked.connect(self.save_to_pdf)

    def get_topics(self):
        move_group_node_info = get_move_group_feedback_topics()
        # params = rospy.client.get_param_names()
        # master = rospy.client.get_master()
        # call_list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        # controllers = call_list_controllers()
        # for controller in controllers.controller:
            # print(controller.name)
        return move_group_node_info

    def init_3d_plot(self):
        # Create a 3D plot
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.figure.tight_layout()
        self.axes = self.figure.add_subplot(111, projection='3d')
        self.plot = None

    def init_dtw_plot(self):
        # Create a plot for the DTW result
        self.dtw_figure = Figure()
        self.dtw_canvas = FigureCanvas(self.dtw_figure)
        self.dtw_figure.tight_layout()
        self.dtw_axes = self.dtw_figure.add_subplot(111)

    def update_user_choices(self):
        if (self.trajectory_carthesian.isChecked() or self.trajectory_joint.isChecked()) and \
           (self.shape_square.isChecked() or self.shape_circle.isChecked() or self.shape_line.isChecked()):
                self.user_choices.run_mode = 'square' if self.shape_square.isChecked() else 'circle' if self.shape_circle.isChecked() else 'line'
                self.user_choices.carthesian = True if self.trajectory_carthesian.isChecked() else False
        # else:
        #     self.user_choices = None  # Reset user_choices if not all options are selected

    def run_program(self):
        tcp_csv = TF_TCP_Base()
        simulation = Simulation(self.user_choices)
    
        success, theoretical_df = simulation.run_path()

        if success:
            measured_df = tcp_csv.csv_to_df()
            self.make_plots(theoretical_df, measured_df)
        
        moveit_commander.roscpp_shutdown()

    def make_plots(self, time_series_1, time_series_2):
        if self.user_choices is not None:
            # Update 3D plot
            if self.plot is not None:
                self.plot.remove()

            dtw_axes = dtw_plot_3d(self.axes, time_series_1, time_series_2)
            self.dtw_canvas.figure = dtw_axes.get_figure()
            self.canvas.draw()
            self.dtw_canvas.draw()
        else:
            error_message = QMessageBox()
            error_message.setIcon(QMessageBox.Critical)
            error_message.setWindowTitle('Error')
            error_message.setText('Please select all options before running DTW.')
            error_message.exec()

    def save_to_pdf(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Plots to PDF", "", "PDF Files (*.pdf);;All Files (*)", options=options)
        if file_name:
            with PdfPages(file_name) as pdf:
                pdf.savefig(self.canvas.figure)
                pdf.savefig(self.dtw_canvas.figure)
    

class UserChoices:
    def __init__(self, run_mode="square", carthesian=True):
        self.run_mode = run_mode
        self.carthesian = carthesian


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TrajectoryDTWApp()
    # window.get_topics()
    window.show()

    # Run the PyQt application
    sys.exit(app.exec_())
