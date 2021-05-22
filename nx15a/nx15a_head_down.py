#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

Ts = 0.1

mlist =[[0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264],
[0.000,0.047,0.172,0.000,-0.040,0.360,0.000,0.047,0.172,0.000,-0.040,0.360],
[0.000,0.004,0.085,0.000,-0.161,0.461,0.000,0.004,0.085,0.000,-0.161,0.461],
[0.000,-0.048,0.003,0.000,-0.281,0.567,0.000,-0.048,0.003,0.000,-0.281,0.567],
[0.000,-0.110,-0.074,0.000,-0.401,0.680,0.000,-0.110,-0.074,0.000,-0.401,0.680],
[0.000,-0.184,-0.145,0.000,-0.523,0.800,0.000,-0.184,-0.145,0.000,-0.523,0.800],
[0.000,-0.110,-0.074,0.000,-0.401,0.680,0.000,-0.110,-0.074,0.000,-0.401,0.680],
[0.000,-0.048,0.003,0.000,-0.281,0.567,0.000,-0.048,0.003,0.000,-0.281,0.567],
[0.000,0.004,0.085,0.000,-0.161,0.461,0.000,0.004,0.085,0.000,-0.161,0.461],
[0.000,0.047,0.172,0.000,-0.040,0.360,0.000,0.047,0.172,0.000,-0.040,0.360],
[0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264]]

def talker():
    rospy.init_node('nx15a_angle_control', anonymous=True)
    pub_lf = rospy.Publisher('/leg_lf_controller/command', JointTrajectory, queue_size=10)
    pub_lr = rospy.Publisher('/leg_lr_controller/command', JointTrajectory, queue_size=10)
    pub_rf = rospy.Publisher('/leg_rf_controller/command', JointTrajectory, queue_size=10)
    pub_rr = rospy.Publisher('/leg_rr_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.5)

    msg_lf = JointTrajectory()
    msg_lr = JointTrajectory()
    msg_rf = JointTrajectory()
    msg_rr = JointTrajectory()
    msg_lf.header.stamp = rospy.Time.now()
    msg_lr.header.stamp = rospy.Time.now()
    msg_rf.header.stamp = rospy.Time.now()
    msg_rr.header.stamp = rospy.Time.now()
    msg_lf.joint_names = [ "joint_LF_1", "joint_LF_2", "joint_LF_3" ]
    msg_lr.joint_names = [ "joint_LR_1", "joint_LR_2", "joint_LR_3" ]
    msg_rf.joint_names = [ "joint_RF_1", "joint_RF_2", "joint_RF_3" ]
    msg_rr.joint_names = [ "joint_RR_1", "joint_RR_2", "joint_RR_3" ]

    msg_lf.points = [JointTrajectoryPoint() for i in range(11)]
    msg_lr.points = [JointTrajectoryPoint() for i in range(11)]
    msg_rf.points = [JointTrajectoryPoint() for i in range(11)]
    msg_rr.points = [JointTrajectoryPoint() for i in range(11)]

    for i in range(0, 11):
        msg_lf.points[i].positions = [mlist[i][0], mlist[i][1], mlist[i][2]]
        msg_lf.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 11):
        msg_lr.points[i].positions = [mlist[i][3], mlist[i][4], mlist[i][5]]
        msg_lr.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 11):
        msg_rf.points[i].positions = [mlist[i][6], mlist[i][7], mlist[i][8]]
        msg_rf.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 11):
        msg_rr.points[i].positions = [mlist[i][9], mlist[i][10], mlist[i][11]]
        msg_rr.points[i].time_from_start = rospy.Time(Ts*(i+1))

    pub_lf.publish(msg_lf)
    pub_lr.publish(msg_lr)
    pub_rf.publish(msg_rf)
    pub_rr.publish(msg_rr)
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass