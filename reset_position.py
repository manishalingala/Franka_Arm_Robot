#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def perform_trajectory():

    rospy.init_node('panda_trajectory_publisher')

    controller_name = '/panda_controller/command'
    trajectory_publisher = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)

    argv = sys.argv[1:]

    panda_joints = ['panda_joint1', 
                    'panda_joint2', 
                    'panda_joint3', 
                    'panda_joint4', 
                    'panda_joint5',
                    'panda_joint6', 
                    'panda_joint7', 
                    'panda_finger_joint1', 
                    'panda_finger_joint2']

    default_positions = [0, 
                         -0.7, 
                         0, 
                         -2.35619449, 
                         0, 
                         1.57079632679, 
                         0.785398163397, 
                         0.001, 
                         0.001]

    goal_positions = [float(arg) if idx < len(argv) else default_positions[idx] for idx, arg in enumerate(default_positions)]

    rospy.loginfo("Goal Position set, let's go!")
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = panda_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for _ in panda_joints]
    trajectory_msg.points[0].accelerations = [0.0 for _ in panda_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)

    trajectory_publisher.publish(trajectory_msg)
if __name__ == '__main__':
    perform_trajectory()