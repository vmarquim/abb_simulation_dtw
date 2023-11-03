#!/usr/bin/env python
import copy

import numpy as np
import pandas as pd
import math

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState, MultiDOFJointState

from tf_tcp_base_publisher import TF_TCP_Base

from kinematics import ForwardKinematics, InverseKinematics
from dtw_plot_3d import dtw_plot_3d


class Simulation:
    def __init__(self, user_choices=None):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "manipulator" #specific to each robot. Check /config/kinematics.yaml
        self.joint_names = ['joint_1', 'joint_2', 'joint_3','joint_4','joint_5', 'joint_6']
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.forwardKinematics = ForwardKinematics()
        self.inverseKinematics = InverseKinematics()
        self.pub = rospy.Publisher("/egm/joint_group_position_controller/command", Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(5)
        
        if user_choices is None:
            self.run_mode = rospy.get_param('/program/run_mode')
            self.carthesian = rospy.get_param('/program/carthesian')
        elif user_choices is not None:
            self.run_mode = user_choices.run_mode
            self.carthesian = user_choices.carthesian

        print("----------------USER CHOICES------------------------")
        print(user_choices)

    
    def run_path(self):
        success, path = False, []
        implemented = ["square", "circle"]
        if self.run_mode not in implemented:
            rospy.loginfo("We can implement new paths here")
        
        if self.run_mode == "square":
            _, path = self.run_square()
            success = True
        
        if self.run_mode == "circle":
            _, path = self.run_circle()
            success = True

        if self.run_mode == "line":
            rospy.loginfo("Not implemented yet")

        ## Implement here new paths - circle, random, etc
        return success, path


    def run_square(self):
        success = False
        current_robot_state = self.robot.get_current_state()
        current_pose = self.forwardKinematics.getPose(self.robot, current_robot_state)
        
        p0 = copy.deepcopy(current_pose)
        
        p1 = copy.deepcopy(p0)
        p1.position.z += 0.15

        p2 = copy.deepcopy(p1)
        p2.position.y += 0.15
        
        p3 = copy.deepcopy(p2)
        p3.position.z -= 0.15

        p4 = copy.deepcopy(p3)
        p4.position.y -= 0.15
        
        step_size = 0.001
        waypoints = [p0, p1, p2, p3, p4]

        path, fraction = self.move_group.compute_cartesian_path(
            waypoints, 
            eef_step=step_size, 
            jump_threshold=0.0
        )

        number_of_points_on_trajectory = len(path.joint_trajectory.points)

        column_values = ["type", "x", "y", "z"]
        carthesian_path = np.empty((number_of_points_on_trajectory, 4))

        for index, point in enumerate(path.joint_trajectory.points):
            carthesian_point = self.get_theoretical_carthesian_point(point)
            carthesian_path[index, :] = [
                1, # 1 für Soll-Bahn, 2 für Ist-Bahn 
                carthesian_point.position.x, 
                carthesian_point.position.y, 
                carthesian_point.position.z, 
            ]

        dataframe_planned_points = pd.DataFrame(data = carthesian_path, columns = column_values)
        
        path.joint_trajectory.points[0].time_from_start = path.joint_trajectory.points[2].time_from_start - rospy.Duration(0, 100) 
        path.joint_trajectory.points[1].time_from_start = path.joint_trajectory.points[2].time_from_start - rospy.Duration(0, 50)

        success = self.move_group.execute(path, wait = True)

        return success, dataframe_planned_points
    

    def run_circle(self):
        success = False
        current_robot_state = self.robot.get_current_state()
        current_pose = self.forwardKinematics.getPose(self.robot, current_robot_state)
        
        p0 = copy.deepcopy(current_pose)
        r = 0.1

        n = 100
        thetas = np.linspace(0, 2*math.pi, n)
        dtheta = 2*math.pi/n
        waypoints = [p0]

        for index, theta in enumerate(thetas):
            dy = -r*math.sin(theta)*dtheta
            dz = r*math.cos(theta)*dtheta
            p_next = copy.deepcopy(waypoints[index])
            p_next.position.y += dy
            p_next.position.z += dz
            waypoints.append(p_next)
        
        step_size = 0.001

        path, fraction = self.move_group.compute_cartesian_path(
            waypoints, 
            eef_step=step_size, 
            jump_threshold=0.0
        )

        number_of_points_on_trajectory = len(path.joint_trajectory.points)

        column_values = ["type", "x", "y", "z"]
        carthesian_path = np.empty((number_of_points_on_trajectory, 4))

        start = rospy.Time.now()

        for index, point in enumerate(path.joint_trajectory.points):
            point.time_from_start = rospy.Time.now() - start
            carthesian_point = self.get_theoretical_carthesian_point(point)
            carthesian_path[index, :] = [
                1, # 1 für Soll-Bahn, 2 für Ist-Bahn 
                carthesian_point.position.x, 
                carthesian_point.position.y, 
                carthesian_point.position.z, 
            ]

        dataframe_planned_points = pd.DataFrame(data = carthesian_path, columns = column_values)

        success = self.move_group.execute(path, wait = True)

        return success, dataframe_planned_points


    def get_theoretical_carthesian_point(self, point): # Victor: Get Points for the Soll-Bahn with the Forward Kinematics transform of the States in the planned Trajectory
        header = Header()
        header.frame_id = "base_link"
        name = self.joint_names
        joint_state = JointState(header, name, point.positions, point.velocities, point.effort)
        multi_dof_joint_state = MultiDOFJointState(header, [], [], [], [])
        attached_collision_objects = []
        robot_state = moveit_msgs.msg.RobotState(joint_state, multi_dof_joint_state, attached_collision_objects, False)

        return self.forwardKinematics.getPose(self.robot, robot_state)


def main():
    rospy.init_node("move_group_test", anonymous=False)
    
    tcp_csv = TF_TCP_Base()
    simulation = Simulation()
    
    success, theoretical_df = simulation.run_path()


    if success:
        measured_df = tcp_csv.csv_to_df()
        dtw_plot_3d(theoretical_df, measured_df)
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
