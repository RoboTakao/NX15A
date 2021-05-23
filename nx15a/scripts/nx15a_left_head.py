#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

Ts = 0.1

mlist =[[0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264,0.000,0.082,0.264],
[0.050,0.053,0.231,-0.049,0.019,0.304,0.049,0.105,0.304,-0.050,0.143,0.231],
[0.102,0.018,0.206,-0.098,-0.046,0.352,0.098,0.123,0.352,-0.102,0.202,0.206],
[0.153,-0.021,0.190,-0.145,-0.112,0.407,0.145,0.134,0.407,-0.153,0.258,0.190],
[0.206,-0.065,0.181,-0.190,-0.179,0.471,0.190,0.139,0.471,-0.206,0.310,0.181],
[0.258,-0.113,0.180,-0.235,-0.248,0.542,0.235,0.137,0.542,-0.258,0.358,0.180],
[0.258,-0.113,0.180,-0.235,-0.248,0.542,0.235,0.137,0.542,-0.258,0.358,0.180],
[0.258,-0.113,0.180,-0.235,-0.248,0.542,0.235,0.137,0.542,-0.258,0.358,0.180],
[0.258,-0.113,0.180,-0.235,-0.248,0.542,0.235,0.137,0.542,-0.258,0.358,0.180],
[0.258,-0.113,0.180,-0.235,-0.248,0.542,0.235,0.137,0.542,-0.258,0.358,0.180],
[0.206,-0.065,0.181,-0.190,-0.179,0.471,0.190,0.139,0.471,-0.206,0.310,0.181],
[0.153,-0.021,0.190,-0.145,-0.112,0.407,0.145,0.134,0.407,-0.153,0.258,0.190],
[0.102,0.018,0.206,-0.098,-0.046,0.352,0.098,0.123,0.352,-0.102,0.202,0.206],
[0.050,0.053,0.231,-0.049,0.019,0.304,0.049,0.105,0.304,-0.050,0.143,0.231],
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

    msg_lf.points = [JointTrajectoryPoint() for i in range(15)]
    msg_lr.points = [JointTrajectoryPoint() for i in range(15)]
    msg_rf.points = [JointTrajectoryPoint() for i in range(15)]
    msg_rr.points = [JointTrajectoryPoint() for i in range(15)]

    for i in range(0, 15):
        msg_lf.points[i].positions = [mlist[i][0], mlist[i][1], mlist[i][2]]
        msg_lf.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 15):
        msg_lr.points[i].positions = [mlist[i][3], mlist[i][4], mlist[i][5]]
        msg_lr.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 15):
        msg_rf.points[i].positions = [mlist[i][6], mlist[i][7], mlist[i][8]]
        msg_rf.points[i].time_from_start = rospy.Time(Ts*(i+1))

    for i in range(0, 15):
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