#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg

import csv

import pandas as pd

from control_msgs.msg import FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult


class TF_TCP_Base:
    def __init__(self):
        # self.listen_tractory_running = rospy.Subscriber(feedback_topic, FollowJointTrajectoryActionFeedback, self.write_to_csv)
        self.listen_tractory_running = rospy.Subscriber("/joint_trajectory_action/feedback", FollowJointTrajectoryActionFeedback, self.write_to_csv)
        
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.target_frame = 'tool0'
        self.source_frame = 'base_link'
        self.rate = rospy.Rate(1)

        self.file = open("test_data.csv", "w")
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(["type", "x", "y", "z", 
                                #   "q1", "q2", "q3", "q4"
                                  ])
        
        
    def write_to_csv(self, data):
        try:
            trans = self.tfBuffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time())

            position = geometry_msgs.msg.Point(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z     
            )
            # orientation = geometry_msgs.msg.Quaternion(
                # trans.transform.rotation.x,
                # trans.transform.rotation.y,
                # trans.transform.rotation.z,
                # trans.transform.rotation.w,
            # )
            
            self.csv_writer.writerow([2, 
                                      position.x, 
                                      position.y, 
                                      position.z, 
                                    #   orientation.x, 
                                    #   orientation.y, 
                                    #   orientation.z, 
                                    #   orientation.w
            ])

            # msg = geometry_msgs.msg.Pose(position, orientation)
            # print(msg)

        except:
            self.rate.sleep()
    
    def csv_to_df(self):
        self.file.close()
        file = open("test_data.csv")
        df = pd.read_csv(file)
        return df
    
class TF_TCP_Base_Test:
    def __init__(self):
        self.file = open("test_data.csv", "w")
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(["type", "x", "y", "z", "q1", "q2", "q3", "q4"])

        self.csv_writer.writerow([2, 1.0, 2.0, 3.0, 0, 0, 0, 0])
        self.csv_writer.writerow([2, 4.0, 5.0, 6.0, 0, 0, 0, 0])
        self.csv_writer.writerow([2, 7.0, 8.0, 9.0, 0, 0, 0, 0])

    def csv_to_df(self):
        self.file.close()
        file = open("test_data.csv")
        df = pd.read_csv(file)
        return df


def main():
    tcp_csv = TF_TCP_Base()
    tcp_csv.csv_to_df()


def main_test():
    tcp_test = TF_TCP_Base_Test()
    tcp_test.csv_to_df()


if __name__ == "__main__":
    main_test()
