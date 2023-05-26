#!/usr/bin/env python

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
"""This python script is the equivalent to the 'src/two_hector_quadrotors_lf.cpp', so it executes the same tasks.
For more information, please refers to that source file.

-- Important: To land the drones and finish this node, please press 'q' key."""

__author__ = "C. Mauricio Arteaga-Escamilla"

import rospy, math
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi

#kbhit function implemented on Linux
import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Define the robots' posture using lists [x,y,z,roll,pitch,yaw]
r_pose1 = [0,0,0,0,0,0]
r_pose2 = [0,0,0,0,0,0]
takeoff_alt = 0.5 #Desired initial altitude

vel1 = Twist() #Create Twist message instance
vel2 = Twist()

t = 0.0; t0 = 0.0; Vxy_max = 2;  Vz_max = 0.5; Wz_max = 4; #timer, initial time, maximum velocities [m/s, rad/s], respectively
T = 100.0; k = 0.1; #Trajectory period, controller gains kx = ky = k

#Leader drone variables
ex = 0.0; ey = 0.0; ez = 0; e_yaw = 0
Vxl = 0; Vyl = 0; Vzl = 0; Wzl = 0 #Leader control signals

#Follower drone variables
exf = 0; eyf = 0; ezf = 0; e_yawf = 0; s_d = 1.5; alp_d = pi; s_zd = 0 #Follower tracking errors and desired formation states
Vxf = 0; Vyf = 0; Vzf = 0; Wzf = 0 #Follower control signals


def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def enable_motors(): #Function to call "/enable_motors" service, needed to move the drone
    #drone 1
    SERVICE_ENABLE_MOTORS = "uav1/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav1: Enable motors successful\n")
        else:
            print("uav1: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")
    
    #drone 2
    SERVICE_ENABLE_MOTORS = "uav2/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav2: Enable motors successful\n")
        else:
            print("uav2: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")

def poseCallback1(msg): #Callback function to get the drone 1 posture
        global r_pose1
        r_pose1[0] = msg.pose.pose.position.x
        r_pose1[1] = msg.pose.pose.position.y
        r_pose1[2] = msg.pose.pose.position.z
        
        #Operations to convert from quaternion to Euler angles
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r_pose1[3], r_pose1[4], r_pose1[5]) = euler_from_quaternion(quater_list) #Euler angles are given in radians

def poseCallback2(msg): #Callback function to get the drone 2 posture
	global r_pose2
	r_pose2[0] = msg.pose.pose.position.x
	r_pose2[1] = msg.pose.pose.position.y
	r_pose2[2] = msg.pose.pose.position.z
	
	#Operations to convert from quaternion to Euler angles
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(r_pose2[3], r_pose2[4], r_pose2[5]) = euler_from_quaternion(quater_list)

def move1(x,y,z,turn): #Function to assign control signals (Vx, Vy, Vz, Wz)
       vel1.linear.x = x; vel1.linear.y = y
       vel1.linear.z = z; vel1.angular.z = turn
       vel_pub1.publish(vel1)

def move2(x,y,z,turn):
	vel2.linear.x = x; vel2.linear.y = y
	vel2.linear.z = z; vel2.angular.z = turn
	vel_pub2.publish(vel2)

def takeoff(): #Takeoff function of all drones
	rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)
	
	#Quadrotor 1
	move1(0,0,0.3,0)	
	
	while(r_pose1[2] < takeoff_alt-0.1):
		vel_pub1.publish(vel1)
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break
	
	move1(0,0,0,0) #Stop
	
	#Quadrotor 2
	move2(0,0,0.3,0)	
	
	while(r_pose2[2] < takeoff_alt-0.1): #Compare if the desired altitude is reached
		vel_pub2.publish(vel2)
		vel_pub1.publish(vel1) #Important: velocities of the drone 1 must keep being published to avoid a not desired landing
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break
	
	move2(0,0,0,0) #Stop

def land(): #Land function of all drones
	print("\n Landing ...\n")
	
	move1(0,0,-0.2,0); move2(0,0,-0.2,0)
		
	while(r_pose1[2] > 0.3 or r_pose2[2] > 0.3):
		vel_pub1.publish(vel1)
		vel_pub2.publish(vel2)
		rate.sleep(); rospy.sleep(0.1)
	
	move1(0,0,0,0); move2(0,0,0,0)

def leader_vel_control(): #Function to generate the desired trajectory and to compute the signals control of the leader robot
	global ex, ey, ez, e_yaw, Vxl, Vyl, vzl, Wzl #Indicate that some variables are global to be used in the main_function
	#Desired trajectory: Helix
	X0 = 0; Y0 = 0; Z0 = 0.5; radius = 3; w = 2*pi/T; Vzd = 0.02

	#Desired position in the 3D space
	Xd = X0+radius*sin(w*t)
	Yd = Y0+radius*cos(w*t)
	Zd = Z0+Vzd*t #linear increasing
	Yawd = sin(w*t) #sinusoidal behavior
	
	#Corresponding time derivatives
	Xdp = radius*w*cos(w*t)
	Ydp = -radius*w*sin(w*t)
	Zdp = Vzd
	Yawdp = w*cos(w*t)
	
	
	ex = r_pose1[0]-Xd; ey = r_pose1[1]-Yd; ez = r_pose1[2]-Zd; e_yaw = r_pose1[5]-Yawd #Compute tracking errors
	
	#Cinematic controller. Auxiliar controls, in global coordinates (local variables)
	Ux = Xdp-k*ex; Uy = Ydp-k*ey
	
	#Translational velocities with respect to the robot frame
	Vxl = Ux*cos(r_pose1[5])+Uy*sin(r_pose1[5])
	Vyl = -Ux*sin(r_pose1[5])+Uy*cos(r_pose1[5])
	
	#Cinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
	Vzl = Zdp-k*ez
	Wzl = Yawdp-k*e_yaw

	#Velocities saturation
	if(abs(Vxl)>Vxy_max):
		Vxl = Vxy_max*abs(Vxl)/Vxl; print("Sat Vxl\t")
	if(abs(Vyl)>Vxy_max):
		Vyl = Vxy_max*abs(Vyl)/Vyl; print("Sat Vyl\t")
	if(abs(Vzl)>Vz_max):
		Vzl = Vz_max*abs(Vzl)/Vzl; print("Sat Vzl\t")
	if(abs(Wzl)>Wz_max):
		Wzl = Wz_max*abs(Wzl)/Wzl; print("Sat Wzl\t")
	
	move1(Vxl,Vyl,Vzl,Wzl) #Publish the 4 control signals

def follower_vel_control(): #Function to compute the signals control of the follower robot
	#Note: desired follower position and yaw orientation are given by the leader's ones
	global exf, eyf, ezf, e_yawf #Indicate that some variables are global to be used in the main_function
	
	#Follower desired position with respect to the gloabl frame
	Xfd = r_pose1[0]+s_d*cos(r_pose1[5]+alp_d)
	Yfd = r_pose1[1]+s_d*sin(r_pose1[5]+alp_d)
	
	#Translational velocities of the leader drone in the global frame
	xlp = Vxl*cos(r_pose1[5])-Vyl*sin(r_pose1[5])
	ylp = Vxl*sin(r_pose1[5])+Vyl*cos(r_pose1[5])

	#Corresponding desired time derivatives
	Xfdp = xlp-Wzl*s_d*sin(r_pose1[5]+alp_d)
	Yfdp = ylp+Wzl*s_d*cos(r_pose1[5]+alp_d)
	
	#Compute tracking errors (with respect to the global frame)
	exf = r_pose2[0]-Xfd; eyf = r_pose2[1]-Yfd
	ezf = r_pose2[2]-(r_pose1[2]+s_zd); e_yawf = r_pose2[5] - r_pose1[5] #Both the altitude and the orientation are equal to leader drone's ones
	
	#Cinematic controller. Auxiliar controls, in global coordinates (local variables)
	Ux = Xfdp-k*exf; Uy = Yfdp-k*eyf
	
	#Translational velocities with respect to the robot frame
	Vxf = Ux*cos(r_pose2[5])+Uy*sin(r_pose2[5])
	Vyf = -Ux*sin(r_pose2[5])+Uy*cos(r_pose2[5])
	
	#Vertical and rotational velocities
	Vzf = Vzl-k*ezf
	Wzf = Wzl-k*e_yawf

	#Velocities saturation
	if(abs(Vxf)>Vxy_max):
		Vxf = Vxy_max*abs(Vxf)/Vxf; print("Sat Vxf\t")
	if(abs(Vyf)>Vxy_max):
		Vyf = Vxy_max*abs(Vyf)/Vyf; print("Sat Vyf\t")
	if(abs(Vzf)>Vz_max):
		Vzf = Vz_max*abs(Vzf)/Vzf; print("Sat Vzf\t")
	if(abs(Wzf)>Wz_max):
		Wzf = Wz_max*abs(Wzf)/Wzf; print("Sat Wzf\t")
	
	move2(Vxf,Vyf,Vzf,Wzf) #Publish the 4 control signals


def main_function():
	rospy.init_node("two_hector_quadrotors_lf", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	global vel_pub1, vel_pub2
	vel_pub1 = rospy.Publisher('uav1/cmd_vel', Twist, queue_size=10) #To publish in the topic
	vel_pub2 = rospy.Publisher('uav2/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('uav1/ground_truth/state', Odometry, poseCallback1) #To subscribe to the topic
	rospy.Subscriber('uav2/ground_truth/state', Odometry, poseCallback2)
	
	
	enable_motors() #Execute the functions
	takeoff()
	
	print("Press 'q' to land the drones and finish the node\n")
	rospy.logwarn(" To start the movement, the simulation must be running\n\n") #Warning message
	
	global t, t0 #t and t0 are global to be used in leaer_vel_control()
	t0 = rospy.Time.now().to_sec()
	
	while(1):
		t = rospy.Time.now().to_sec()-t0
		leader_vel_control() #Compute the control signals of the leader robot
		follower_vel_control() #Compute the control signals of the follower robot
		
		if counter == 25: #Frequency divisor
			rospy.loginfo("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw)
			rospy.loginfo("exf: %.3f eyf: %.3f ezf: %.2f e_yawf: %.3f\n", exf,eyf,ezf,e_yawf)
			counter = 0 #Reset the counter
		else:
			counter += 1
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'q'):
			break

	land() #Execute land function
	print("\n Node finished\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
    	main_function()  #Execute the function
	
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
