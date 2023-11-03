import rospy
import moveit_msgs


class ForwardKinematics():
    def __init__(self):
        # prepare service for forward kinematics calculation
        self.fk_srv = rospy.ServiceProxy('/compute_fk', moveit_msgs.srv.GetPositionFK)
        # wait for service to become available
        self.fk_srv.wait_for_service()
        rospy.loginfo('GetPositionFK service is available')


    def getPose(self, robot, robot_state, link="tool0",reference_frame_id="base_link"):
        '''
        Calculate the pose of the link related to the 
        frame reference_frame_id for the given 
        robot state robot_state.
        Return type is geometry_msgs.msg.Pose
        '''
        # prepare request
        gfkr = moveit_msgs.srv.GetPositionFKRequest()
        gfkr.header.frame_id = reference_frame_id
        gfkr.fk_link_names = robot.get_link_names()
        gfkr.robot_state = robot_state

        # call service
        result = self.fk_srv.call(gfkr)

        for link_name,pose_stamped in zip(result.fk_link_names,result.pose_stamped):
            if link_name == link:
                return pose_stamped.pose


class InverseKinematics():
    def __init__(self):
        # prepare service for inverse kinematics calculation
        self.ik_srv = rospy.ServiceProxy('/compute_ik', moveit_msgs.srv.GetPositionIK)
        # wait for service to become available
        self.ik_srv.wait_for_service()
        rospy.loginfo('GetPositionIK service is available')


    def getJointAngles(self, goal, current_robot_state, group_name='manipulator',link="tool0"):
        '''
        Calculate the joint angles for the given goal.
        current_robot_state is the seed state for the IK solver.
        Return type is moveit_msgs.msg.RobotState
        '''
        # prepare request
        gikr = moveit_msgs.srv.GetPositionIKRequest()
        gikr.ik_request.group_name = group_name
        current_robot_state.is_diff = True
        gikr.ik_request.robot_state = current_robot_state
        gikr.ik_request.ik_link_name = link
        gikr.ik_request.avoid_collisions = False
        gikr.ik_request.pose_stamped = goal
        gikr.ik_request.timeout = rospy.Duration(1.0)

        # call service
        result = self.ik_srv.call(gikr)
        if(result.error_code.val != 1):
            print("Error: ", result.error_code.val)

        return result.solution
