#!/usr/bin/env python

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
"""This python script is the equivalent to the 'src/hector_quadrotor_controller_paths.cpp', so it executes the same tasks.
For more information, please refers to that source file.

-- Important: To land the drone and finish this node, please press 'q' key."""

__author__ = "C. Mauricio Arteaga-Escamilla"

import rospy, math
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from nav_msgs.msg import Odometry, Path #Message type to subscribe to /odom and to publish in /path topics
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi
from geometry_msgs.msg import PoseStamped #Message type to publish the robots' path

#kbhit function implemented on Linux
import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Define global variables (position and orientation, using Euler anlges)
x=0; y=0; z=0; roll=0; pitch=0; yaw=0
takeoff_alt = 1.2 #Desired initial altitude

vel = Twist() #Create Twist and Path message instances
drone_path_msg = Path()
tr_path_msg = Path()

#Create the title of the global frame. Global frame is required by Rviz to define the SAME frame for any robot
global_frame = "/world"

t = 0.0; t0 = 0.0; Vxy_max = 1.5;  Vz_max = 0.5; Wz_max = 4; #timer, initial time, maximum velocities [m/s, rad/s], respectively
T = 100.0; k = 0.1; #Trajectory period, controller gains kx = ky = k
ex = 0.0; ey = 0.0; ez = 0; e_yaw = 0

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
    SERVICE_ENABLE_MOTORS = "enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("Enable motors successful\n")
        else:
            print("Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")

def poseCallback(msg): #Callback function to get the drone posture
        global x, y, z, roll, pitch, yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        #Operations to convert from quaternion to Euler angles
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, yaw) = euler_from_quaternion(quater_list)
        
        drone_pose = PoseStamped() #Create the PoseStamped instance
        drone_pose.pose.position = msg.pose.pose.position
        drone_path_msg.poses.append(drone_pose) #Add the drone position to the list of points

def movement(x,y,z,turn): #Function to assign control signals (Vx, Vy, Vz, Wz)
	vel.linear.x = x; vel.linear.y = y
	vel.linear.z = z; vel.angular.z = turn

def takeoff(): #Takeoff function
	movement(0,0,0.5,0)
	velocity_publisher.publish(vel)
	
	rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)
	while(z < takeoff_alt-0.1):
		velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == b'\x1b'):
			break
	
	movement(0,0,0,0)
	velocity_publisher.publish(vel)

def land(): #Land function
	movement(0,0,-0.2,0)
	velocity_publisher.publish(vel)
	print("\n Landing ...\n")
	
	while(z > 0.3):
		velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)
	
	movement(0,0,0,0)
	velocity_publisher.publish(vel)

def velocity_controller(): #Function to generate the desired trajectory and to compute the control signals
	global ex, ey, ez, e_yaw #Indicate that some variables are global to be used in the main_function-----
	#Desired trajectory: Lemniscate
	a = 3; b = 1.5; X0 = 0; Y0 = -0.5; Z0 = 1.5; w = 2*pi/T; c = 0.5; d = pi/2
	#Desired position in the 3D space 
	Xd = X0+a*sin(w*t)
	Yd = Y0+b*sin(2*w*t)
	Zd = Z0+c*sin(w*t) #Note: 1 <= Zd <= 2
	Yawd = d*sin(w*t)
	
	#Corresponding time derivatives
	Xdp = a*w*cos(w*t)
	Ydp = 2*b*w*cos(2*w*t)
	Zdp = c*w*cos(w*t)
	Yawdp = d*w*cos(w*t)
	
	#Corresponding time derivatives
	Xdp = a*w*cos(w*t)
	Ydp = 2*b*w*cos(2*w*t)
	Zdp = c*w*cos(w*t)
	Yawdp = d*w*cos(w*t)
	
	
	#Assign the desired position to "tr_pose"
	tr_pose = PoseStamped() #Create the PoseStamped instance
	tr_pose.pose.position.x = Xd; tr_pose.pose.position.y = Yd
	tr_pose.pose.position.z = Zd 
	tr_path_msg.poses.append(tr_pose) #Add the new point to the list
	
	
	ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw-Yawd #Compute tracking errors
	
	#Cinematic controller. Auxiliar controls, in global coordinates
	Ux = Xdp-k*ex; Uy = Ydp-k*ey
	
	#Translational velocities with respect to the robot frame
	Vx = Ux*cos(yaw)+Uy*sin(yaw)
	Vy = -Ux*sin(yaw)+Uy*cos(yaw)
	
	#Cinematic controller. Note: Vz and Wz are compute directly since they are given with respect to the robot frame
	Vz = Zdp-k*ez
	Wz = Yawdp-k*e_yaw

	#Velocities saturation
	if(abs(Vx)>Vxy_max):
		Vx = Vxy_max*abs(Vx)/Vx; print("Sat Vx\t")
	if(abs(Vy)>Vxy_max):
		Vy = Vxy_max*abs(Vy)/Vy; print("Sat Vy\t")
	if(abs(Vz)>Vz_max):
		Vz = Vz_max*abs(Vz)/Vz; print("Sat Vz\t")
	if(abs(Wz)>Wz_max):
		Wz = Wz_max*abs(Wz)/Wz; print("Sat Wz\t")
	
	movement(Vx,Vy,Vz,Wz)
	velocity_publisher.publish(vel) #Publish the 4 control signals


def main_function():
	rospy.init_node("hector_quadrotor_controller_paths", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	global velocity_publisher
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
	
	#To publish the paths
	drone_path_pub = rospy.Publisher('/path', Path, queue_size=10)
	tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10) 
	
	global drone_path_msg, tr_path_msg
	#Important: Assignation of the SAME reference frame for ALL robots
	drone_path_msg.header.frame_id = tr_path_msg.header.frame_id = global_frame

	enable_motors() #Execute the functions
	takeoff()
	
	print("Press 'q' to land the drone and finish the node\n")
	print("To clean up both paths, press 'c' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n") #Warning message
	
	global t, t0 #t and t0 are global to be used in velocity_controller()
	t0 = rospy.Time.now().to_sec()
	
	while(1):
		t = rospy.Time.now().to_sec()-t0
		velocity_controller() #Compute the control signals
		
		if counter == 50: #Frequency divisor
			rospy.loginfo("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw) #Print tracking errors [m] and orientation error [rad]
			counter = 0 #Reset the counter
		else:
			counter += 1
		
		#Publish the "path" messages
		drone_path_pub.publish(drone_path_msg); tr_path_pub.publish(tr_path_msg)
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'c'): #Clear paths lists
			del drone_path_msg.poses[:]; del tr_path_msg.poses[:]
			rospy.logwarn("Clear paths\n")
			key = 0; #Reset the key
		elif(key == 'q'):
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
