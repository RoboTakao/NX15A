#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import math
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

pi = math.pi

L1 = 71
L2 = 70
Z0 = 39.4
L0 = 9.7
Nee = 25.46

H0 = 100 #Height
Zc = 25 #Half Width
Xp = 72.75 #Half pitch

angXX0 = 0.0
angZZ0 = 0.0
angXX = 0.0
angYY = 0.0
angZZ = 0.0
div_angXX = 0.0
div_angZZ = 0.0
shiftXX0 = 5.0
shiftXX = 0.0
shiftYY = 0.0
shiftZZ = 0.0

flag_w_s = False
flag_m = False
cnt_w_s = 0
cnt_m = 0

Distance = 0.0
Dis_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

fup = 25.0

Ts = 0.05
Ts_h = 0.3

#initial coordinate
FtCoLF0 = np.array([-Xp + shiftXX0, H0, Z0 + Zc])     # Left Front X,Y,Z
FtCoLR0 = np.array([Xp + shiftXX0, H0, Z0 + Zc])      # Left Rear X,Y,Z
FtCoRF0 = np.array([-Xp + shiftXX0, H0, -(Z0 + Zc)])  # Right Front X,Y,Z
FtCoRR0 = np.array([Xp + shiftXX0, H0, -(Z0 + Zc)])   # Right Rear X,Y,Z

mlist0_h = [-(FtCoLF0[0]+Xp), FtCoLF0[1], FtCoLF0[2]-Zc, \
    -(FtCoLR0[0]-Xp), FtCoLR0[1], FtCoLR0[2]-Zc, \
    -(FtCoRF0[0]+Xp), FtCoRF0[1], -(FtCoRF0[2]+Zc), \
    (FtCoRR0[0]-Xp), FtCoRR0[1], -(FtCoRR0[2]+Zc)]

def callback(wii_joy):
    global angXX, angYY, angZZ, shiftXX, shiftYY, shiftZZ, shiftXX0, flag_w_s, flag_m, div_angZZ, cnt_w_s, cnt_m, Distance, Dis_array

    print("Wii", wii_joy.data[0], wii_joy.data[1], wii_joy.data[2], wii_joy.data[3])

    if wii_joy.data[0] == 1:
        if cnt_w_s == 3:
            flag_w_s = not flag_w_s
            cnt_w_s = 0
        else:
            cnt_w_s += 1
    
    if wii_joy.data[1] == 1:
        if cnt_m == 3:
            flag_m = not flag_m
            cnt_m = 0
        else:
            cnt_m += 1

    if flag_w_s == False:
        if flag_m == False:
            angYY = float(wii_joy.data[2])
            angZZ = -float(wii_joy.data[3])
            div_angZZ = angZZ - angZZ0
            print("Stay Control Mode", flag_w_s, flag_m, angYY, angZZ)
        if flag_m == True:
            shiftXX0 = shiftXX0 + float(wii_joy.data[3])/5
            if shiftXX0 >= 10:
                shiftXX0 = 10
            if shiftXX0 <= -10:
                shiftXX0 = -10
            shiftYY = shiftYY + float(wii_joy.data[2])/5
            if shiftYY >= 20:
                shiftYY = 20
            if shiftYY <= -20:
                shiftYY = -20
            print("Stay Shift Mode", flag_w_s, flag_m, shiftXX0, shiftYY)
    
    if flag_w_s == True:
        if Distance >= 180.0:
            shiftXX = -5.0
            angYY = 0.0
        if Distance < 180.0:
            flag_w_s = False
            for i in range(17):
                angYY = float(i*2-16)
                #talker(shiftXX, shiftYY, shiftZZ, div_angXX, angYY, div_angZZ) # Without UMU Data in thid time
                talker(shiftXX, shiftYY, shiftZZ, angXX, angYY, angZZ)
                Dis_array[i] = Distance
                rospy.sleep(0.1)
            angYY = 0.0
            print("Distance array", Dis_array)
            #talker(shiftXX, shiftYY, shiftZZ, div_angXX, angYY, div_angZZ) # Without UMU Data in thid time
            talker(shiftXX, shiftYY, shiftZZ, angXX, angYY, angZZ)
            flag_w_s = True
            Distance_L = 0
            Distance_R = 0
            for k in range(6):
                Distance_L += Dis_array[k]
                Distance_R += Dis_array[11 + k]
            if Distance_L > Distance_R:
                angYY = 5.0
            else:
                angYY = -5.0
            talker(shiftXX, shiftYY, shiftZZ, angXX, angYY, angZZ)
            rospy.sleep(0.8)
            angYY = 0.0
        
def callback_imu(IMU_Distance_data):
    global angXX, angYY, angXX0, angZZ0, div_angXX, div_angZZ, Distance
    #print("IMU_Distance" ,IMU_Distance_data.data[0], IMU_Distance_data.data[1], IMU_Distance_data.data[2], IMU_Distance_data.data[3])

    if flag_w_s == False:
        angXX0 = -IMU_Distance_data.data[0]
        angZZ0 = -IMU_Distance_data.data[1]
        div_angXX = angXX - angXX0
        div_angZZ = angZZ - angZZ0

    Distance = IMU_Distance_data.data[3]

def transform(FtCo, angleXYZ):
    RotX = np.array([[1, 0, 0], [0, math.cos(angleXYZ[0]), -math.sin(angleXYZ[0])], [0, math.sin(angleXYZ[0]), math.cos(angleXYZ[0])]])
    RotY = np.array([[math.cos(angleXYZ[1]), 0, math.sin(angleXYZ[1])], [0, 1, 0], [-math.sin(angleXYZ[1]), 0, math.cos(angleXYZ[1])]])
    RotZ = np.array([[math.cos(angleXYZ[2]), -math.sin(angleXYZ[2]), 0], [math.sin(angleXYZ[2]), math.cos(angleXYZ[2]), 0], [0, 0, 1]])

    return np.dot(RotX,np.dot(RotY,np.dot(RotZ, FtCo)))

def inverse_kinematic(X2, H2, Z2):
    Alfa = math.atan2(Z2, H2)
    L = H2/math.cos(Alfa)
    Y2 = Z0 * math.tan(math.acos(Z0/L)) - L0
    Theta30 = math.atan2(Z0, Y2+L0)
    Theta3 = Alfa - Theta30
    
    Theta1 = math.acos((X2**2 + Y2**2 + L1**2 - L2**2)/(2 * L1 * math.sqrt(X2**2 + Y2**2))) + math.atan2(Y2, X2)
    Theta2 = math.atan2(Y2 - L1 * math.sin(Theta1), X2 - L1 * math.cos(Theta1)) - Theta1

    Theta1_2 = Theta1 - pi*3/4
    Theta2_2 = Theta2 + (90 + Nee)/180*pi

    return Theta3, Theta1_2, Theta2_2

def talker(shiftX, shiftY, shiftZ, angX, angY, angZ):
    global mlist0_h, shiftXX0, fup, flag_w_s
    shift = np.array([shiftX, 0, shiftZ])
    shift0 = np.array([shiftXX0, 0, 0])
    euler = np.array([angX, angY, angZ]) / 180 * pi  # X axis, Y axis , Z axis

    #initial coordinate
    FtCoLF0 = np.array([-Xp + shiftXX0, H0 + shiftY, Z0 + Zc])     # Left Front X,Y,Z
    FtCoLR0 = np.array([Xp + shiftXX0, H0 + shiftY, Z0 + Zc])      # Left Rear X,Y,Z
    FtCoRF0 = np.array([-Xp + shiftXX0, H0 + shiftY, -(Z0 + Zc)])  # Right Front X,Y,Z
    FtCoRR0 = np.array([Xp + shiftXX0, H0 + shiftY, -(Z0 + Zc)])   # Right Rear X,Y,Z

    #step1 coordinate
    FtCoLFtf0 = transform(FtCoLF0, euler) + shift + shift0 # Left Front X,Y,Z
    FtCoLRtf0 = transform(FtCoLR0, euler) + shift + shift0 # Left Rear X,Y,Z
    FtCoRFtf0 = transform(FtCoRF0, euler) + shift + shift0 # Right Front X,Y,Z
    FtCoRRtf0 = transform(FtCoRR0, euler) + shift + shift0 # Right Rear X,Y,Z

    #step2 coordinate
    FtCoLFtf1 = transform(FtCoLF0, -euler) - shift + shift0 # Left Front X,Y,Z
    FtCoLRtf1 = transform(FtCoLR0, -euler) - shift + shift0 # Left Rear X,Y,Z
    FtCoRFtf1 = transform(FtCoRF0, -euler) - shift + shift0 # Right Front X,Y,Z
    FtCoRRtf1 = transform(FtCoRR0, -euler) - shift + shift0 # Right Rear X,Y,Z

    #step3 coordinate
    FtCoLFtf2 = transform(FtCoLF0, -2*euler) - 2*shift + shift0 # Left Front X,Y,Z
    FtCoLRtf2 = transform(FtCoLR0, -2*euler) - 2*shift + shift0 # Left Rear X,Y,Z
    FtCoRFtf2 = transform(FtCoRF0, -2*euler) - 2*shift + shift0 # Right Front X,Y,Z
    FtCoRRtf2 = transform(FtCoRR0, -2*euler) - 2*shift + shift0 # Right Rear X,Y,Z

    mlist0 = [-(FtCoLF0[0]+Xp), FtCoLF0[1], FtCoLF0[2]-Zc, \
    -(FtCoLR0[0]-Xp), FtCoLR0[1], FtCoLR0[2]-Zc, \
    -(FtCoRF0[0]+Xp), FtCoRF0[1], -(FtCoRF0[2]+Zc), \
    -(FtCoRR0[0]-Xp), FtCoRR0[1], -(FtCoRR0[2]+Zc)]

    mlist1 = [-(FtCoLF0[0]+Xp), FtCoLF0[1], FtCoLF0[2]-Zc, \
    -(FtCoLR0[0]-Xp), FtCoLR0[1] - fup, FtCoLR0[2]-Zc, \
    -(FtCoRF0[0]+Xp), FtCoRF0[1] - fup, -(FtCoRF0[2]+Zc), \
    -(FtCoRR0[0]-Xp), FtCoRR0[1], -(FtCoRR0[2]+Zc)]

    mlist2 = [-(FtCoLFtf1[0]+Xp), FtCoLRtf1[1], FtCoLFtf1[2]-Zc, \
    -(FtCoLRtf0[0]-Xp), FtCoLRtf0[1], FtCoLRtf0[2]-Zc, \
    -(FtCoRFtf0[0]+Xp), FtCoRFtf0[1], -(FtCoRFtf0[2]+Zc), \
    -(FtCoRRtf1[0]-Xp), FtCoRRtf1[1], -(FtCoRRtf1[2]+Zc)]

    mlist3 = [-(FtCoLFtf2[0]+Xp), FtCoLRtf2[1], FtCoLFtf2[2]-Zc, \
    -(FtCoLR0[0]-Xp), FtCoLR0[1], FtCoLR0[2]-Zc, \
    -(FtCoRF0[0]+Xp), FtCoRF0[1], -(FtCoRF0[2]+Zc), \
    -(FtCoRRtf2[0]-Xp), FtCoRRtf2[1], -(FtCoRRtf2[2]+Zc)]
    
    mlist4 = [-(FtCoLF0[0]+Xp), FtCoLF0[1] - fup, FtCoLF0[2]-Zc, \
    -(FtCoLR0[0]-Xp), FtCoLR0[1], FtCoLR0[2]-Zc, \
    -(FtCoRF0[0]+Xp), FtCoRF0[1], -(FtCoRF0[2]+Zc), \
    -(FtCoRR0[0]-Xp), FtCoRR0[1] - fup, -(FtCoRR0[2]+Zc)]

    mlist5 = [-(FtCoLFtf0[0]+Xp), FtCoLRtf0[1], FtCoLFtf0[2]-Zc, \
    -(FtCoLRtf1[0]-Xp), FtCoLRtf1[1], FtCoLRtf1[2]-Zc, \
    -(FtCoRFtf1[0]+Xp), FtCoRFtf1[1], -(FtCoRFtf1[2]+Zc), \
    -(FtCoRRtf0[0]-Xp), FtCoRRtf0[1], -(FtCoRRtf0[2]+Zc)]

    mlist6 = [-(FtCoLF0[0]+Xp), FtCoLF0[1], FtCoLF0[2]-Zc, \
    -(FtCoLRtf2[0]-Xp), FtCoLR0[1], FtCoLRtf2[2]-Zc, \
    -(FtCoRFtf2[0]+Xp), FtCoRF0[1], -(FtCoRFtf2[2]+Zc), \
    -(FtCoRR0[0]-Xp), FtCoRR0[1], -(FtCoRR0[2]+Zc)]

    mlist1_h = [-(FtCoLFtf0[0]+Xp), FtCoLFtf0[1], FtCoLFtf0[2]-Zc, \
    -(FtCoLRtf0[0]-Xp), FtCoLRtf0[1], FtCoLRtf0[2]-Zc, \
    -(FtCoRFtf0[0]+Xp), FtCoRFtf0[1], -(FtCoRFtf0[2]+Zc), \
    -(FtCoRRtf0[0]-Xp), FtCoRRtf0[1], -(FtCoRRtf0[2]+Zc)]

    mlist = [[mlist0[0], mlist0[1], mlist0[2], mlist0[3], mlist0[4], mlist0[5], mlist0[6], mlist0[7], mlist0[8], mlist0[9], mlist0[10], mlist0[11]],
            [mlist1[0], mlist1[1], mlist1[2], mlist1[3], mlist1[4], mlist1[5], mlist1[6], mlist1[7], mlist1[8], mlist1[9], mlist1[10], mlist1[11]],
            [mlist2[0], mlist2[1], mlist2[2], mlist2[3], mlist2[4], mlist2[5], mlist2[6], mlist2[7], mlist2[8], mlist2[9], mlist2[10], mlist2[11]],
            [mlist3[0], mlist3[1], mlist3[2], mlist3[3], mlist3[4], mlist3[5], mlist3[6], mlist3[7], mlist3[8], mlist3[9], mlist3[10], mlist3[11]],
            [mlist4[0], mlist4[1], mlist4[2], mlist4[3], mlist4[4], mlist4[5], mlist4[6], mlist4[7], mlist4[8], mlist4[9], mlist4[10], mlist4[11]],
            [mlist5[0], mlist5[1], mlist5[2], mlist5[3], mlist5[4], mlist5[5], mlist5[6], mlist5[7], mlist5[8], mlist5[9], mlist5[10], mlist5[11]],
            [mlist6[0], mlist6[1], mlist6[2], mlist6[3], mlist6[4], mlist6[5], mlist6[6], mlist6[7], mlist6[8], mlist6[9], mlist6[10], mlist6[11]],
            [mlist1[0], mlist1[1], mlist1[2], mlist1[3], mlist1[4], mlist1[5], mlist1[6], mlist1[7], mlist1[8], mlist1[9], mlist1[10], mlist1[11]],
            [mlist2[0], mlist2[1], mlist2[2], mlist2[3], mlist2[4], mlist2[5], mlist2[6], mlist2[7], mlist2[8], mlist2[9], mlist2[10], mlist2[11]],
            [mlist3[0], mlist3[1], mlist3[2], mlist3[3], mlist3[4], mlist3[5], mlist3[6], mlist3[7], mlist3[8], mlist3[9], mlist3[10], mlist3[11]],
            [mlist4[0], mlist4[1], mlist4[2], mlist4[3], mlist4[4], mlist4[5], mlist4[6], mlist4[7], mlist4[8], mlist4[9], mlist4[10], mlist4[11]],
            [mlist5[0], mlist5[1], mlist5[2], mlist5[3], mlist5[4], mlist5[5], mlist5[6], mlist5[7], mlist5[8], mlist5[9], mlist5[10], mlist5[11]],
            [mlist6[0], mlist6[1], mlist6[2], mlist6[3], mlist6[4], mlist6[5], mlist6[6], mlist6[7], mlist6[8], mlist6[9], mlist6[10], mlist6[11]],
            [mlist1[0], mlist1[1], mlist1[2], mlist1[3], mlist1[4], mlist1[5], mlist1[6], mlist1[7], mlist1[8], mlist1[9], mlist1[10], mlist1[11]],
            [mlist0[0], mlist0[1], mlist0[2], mlist0[3], mlist0[4], mlist0[5], mlist0[6], mlist0[7], mlist0[8], mlist0[9], mlist0[10], mlist0[11]]]

    mlist_h = [[mlist0_h[0], mlist0_h[1], mlist0_h[2], mlist0_h[3], mlist0_h[4], mlist0_h[5], mlist0_h[6], mlist0_h[7], mlist0_h[8], mlist0_h[9], mlist0_h[10], mlist0_h[11]],
            [mlist1_h[0], mlist1_h[1], mlist1_h[2], mlist1_h[3], mlist1_h[4], mlist1_h[5], mlist1_h[6], mlist1_h[7], mlist1_h[8], mlist1_h[9], mlist1_h[10], mlist1_h[11]]]
    mlist0_h = mlist1_h

    rospy.init_node('nx15a_angle_control', anonymous=True)
    pub_lf = rospy.Publisher('/leg_lf_controller/command', JointTrajectory, queue_size=10)
    pub_lr = rospy.Publisher('/leg_lr_controller/command', JointTrajectory, queue_size=10)
    pub_rf = rospy.Publisher('/leg_rf_controller/command', JointTrajectory, queue_size=10)
    pub_rr = rospy.Publisher('/leg_rr_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.2)

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

    if flag_w_s == True:
        msg_lf.points = [JointTrajectoryPoint() for i in range(15)]
        msg_lr.points = [JointTrajectoryPoint() for i in range(15)]
        msg_rf.points = [JointTrajectoryPoint() for i in range(15)]
        msg_rr.points = [JointTrajectoryPoint() for i in range(15)]

        for i in range(0, 15):
            return_lf = inverse_kinematic(mlist[i][0], mlist[i][1], mlist[i][2])
            msg_lf.points[i].positions = [-return_lf[0], return_lf[1], return_lf[2]]
            msg_lf.points[i].time_from_start = rospy.Time(Ts*i)

        for i in range(0, 15):
            return_lr = inverse_kinematic(mlist[i][3], mlist[i][4], mlist[i][5])
            msg_lr.points[i].positions = [-return_lr[0], return_lr[1], return_lr[2]]
            msg_lr.points[i].time_from_start = rospy.Time(Ts*i)

        for i in range(0, 15):
            return_rf = inverse_kinematic(mlist[i][6], mlist[i][7], mlist[i][8])
            msg_rf.points[i].positions = [return_rf[0], return_rf[1], return_rf[2]]
            msg_rf.points[i].time_from_start = rospy.Time(Ts*i)

        for i in range(0, 15):
            return_rr = inverse_kinematic(mlist[i][9], mlist[i][10], mlist[i][11])
            msg_rr.points[i].positions = [return_rr[0], return_rr[1], return_rr[2]]
            msg_rr.points[i].time_from_start = rospy.Time(Ts*i)
    
    if flag_w_s == False:
        msg_lf.points = [JointTrajectoryPoint() for i in range(2)]
        msg_lr.points = [JointTrajectoryPoint() for i in range(2)]
        msg_rf.points = [JointTrajectoryPoint() for i in range(2)]
        msg_rr.points = [JointTrajectoryPoint() for i in range(2)]

        for i in range(0, 2):
            return_lf = inverse_kinematic(mlist_h[i][0], mlist_h[i][1], mlist_h[i][2])
            msg_lf.points[i].positions = [-return_lf[0], return_lf[1], return_lf[2]]
            msg_lf.points[i].time_from_start = rospy.Time(Ts_h*i)

        for i in range(0, 2):
            return_lr = inverse_kinematic(mlist_h[i][3], mlist_h[i][4], mlist_h[i][5])
            msg_lr.points[i].positions = [-return_lr[0], return_lr[1], return_lr[2]]
            msg_lr.points[i].time_from_start = rospy.Time(Ts_h*i)

        for i in range(0, 2):
            return_rf = inverse_kinematic(mlist_h[i][6], mlist_h[i][7], mlist_h[i][8])
            msg_rf.points[i].positions = [return_rf[0], return_rf[1], return_rf[2]]
            msg_rf.points[i].time_from_start = rospy.Time(Ts_h*i)

        for i in range(0, 2):
            return_rr = inverse_kinematic(mlist_h[i][9], mlist_h[i][10], mlist_h[i][11])
            msg_rr.points[i].positions = [return_rr[0], return_rr[1], return_rr[2]]
            msg_rr.points[i].time_from_start = rospy.Time(Ts_h*i)

    pub_lf.publish(msg_lf)
    pub_lr.publish(msg_lr)
    pub_rf.publish(msg_rf)
    pub_rr.publish(msg_rr)
    rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.Subscriber("Wii_joystick",Int32MultiArray,callback)
    rospy.Subscriber("IMU_Distance_data",Int32MultiArray,callback_imu)
    
    while True:
        try:
            #talker(shiftXX, shiftYY, shiftZZ, div_angXX, angYY, div_angZZ) # Without UMU Data in thid time
            talker(shiftXX, shiftYY, shiftZZ, angXX, angYY, angZZ)
        except rospy.ROSInterruptException: pass