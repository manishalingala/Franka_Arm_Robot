#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from panda_kinematics import PandaWithHandKinematics

def perform_trajectory():
    rospy.init_node('panda_trajectory_publisher')
    controller_name = '/panda_controller/command'
    trajectory_publisher = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)

    fr3_joints = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5',
                    'fr3_joint6', 'fr3_joint7', 'fr3_finger_joint1', 'fr3_finger_joint2']
    
    kinematics = PandaWithHandKinematics()
    position = np.array([0.5, 0.0, 0.4])
    orientation_quat = np.array([ 0.0, 1.0, 0.0, 0.0]) # xyzw
    initial_joint_positions = np.array([0, 0 , 0, 0, 0, 0,0])
    solution = kinematics.ik(initial_joint_positions, position, orientation_quat)

    goal_positions = np.append(solution , [0, 0])

    rospy.loginfo("Joint Angles Solution: %s", solution)
    rospy.loginfo("Goal Position set, let's go!")
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = fr3_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = list(goal_positions) 
    trajectory_msg.points[0].velocities = [0.0 for _ in fr3_joints]
    trajectory_msg.points[0].accelerations = [0.0 for _ in fr3_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publisher.publish(trajectory_msg)

if __name__ == '__main__':
    perform_trajectory()