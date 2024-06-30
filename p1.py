#!/usr/bin/env python
import numpy as np
import rospy
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped

import sys, select, os  # Handling command line arguments
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():  # Function to use keyboard events on Linux
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

kp = [0.25, 0.25, 0.5, 1.5]
kd = [0.05, 0.05, 0.05, 0]
ki = [0, 0, 0, 0]

class Drone(object):
    def __init__(self):
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.desired_pos = Point()
        self.position_error = Point()

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/drone/position', Point, queue_size=10)
        self.desired_pose_pub = rospy.Publisher('/drone/desired_position', Point, queue_size=10)
        self.error_pub = rospy.Publisher('/drone/position_error', Point, queue_size=10)
        
        # Publishers for velocities
        self.vel_linear_x_pub = rospy.Publisher('/cmd_vel/linear/x', Float32, queue_size=10)
        self.vel_linear_y_pub = rospy.Publisher('/cmd_vel/linear/y', Float32, queue_size=10)
        self.vel_angular_z_pub = rospy.Publisher('/cmd_vel/angular/z', Float32, queue_size=10)

        # Publishers for visualization in rqt_plot
        self.thrust_pub = rospy.Publisher('/visualize/thrust', Float32, queue_size=10)
        self.torque_x_pub = rospy.Publisher('/visualize/torque_x', Float32, queue_size=10)
        self.torque_y_pub = rospy.Publisher('/visualize/torque_y', Float32, queue_size=10)
        self.torque_z_pub = rospy.Publisher('/visualize/torque_z', Float32, queue_size=10)

        # Subscribe to the /command/wrench topic
        rospy.Subscriber('/command/wrench', WrenchStamped, self.wrench_callback)

        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()  # Enable motors when initializing the drone
        self.takeoff_complete = False  # Flag to indicate if takeoff is complete
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]

        self.thrust = 0.0
        self.torque_x = 0.0
        self.torque_y = 0.0
        self.torque_z = 0.0

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)
        self.publish_position()

    def publish_position(self):
        point = Point(x=self.cord[0], y=self.cord[1], z=self.cord[2])
        self.pose_pub.publish(point)
        self.desired_pose_pub.publish(Point(x=self.desired_pos.x, y=self.desired_pos.y))
        self.error_pub.publish(Point(x=self.position_error.x, y=self.position_error.y))

    def publish_velocities(self):
        # Publish current velocities
        self.vel_linear_x_pub.publish(self.vel.linear.x)
        self.vel_linear_y_pub.publish(self.vel.linear.y)
        self.vel_angular_z_pub.publish(self.vel.angular.z)

    def wrench_callback(self, msg):
        # Extract force and torque values from the Wrench message
        thrust = msg.wrench.force.z
        torque_x = msg.wrench.torque.x
        torque_y = msg.wrench.torque.y
        torque_z = msg.wrench.torque.z

        # Store the values
        self.thrust = thrust
        self.torque_x = torque_x
        self.torque_y = torque_y
        self.torque_z = torque_z

        # Publish the values for visualization
        self.publish_values()

    def publish_values(self):
        self.thrust_pub.publish(self.thrust)
        self.torque_x_pub.publish(self.torque_x)
        self.torque_y_pub.publish(self.torque_y)
        self.torque_z_pub.publish(self.torque_z)

    def enable_motors(self):
        rospy.wait_for_service('/enable_motors')
        enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            # If takeoff is not complete, gradually ascend to 2 meters
            if self.cord[2] < 2:
                self.vel.linear.z = 0.25
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True  # Mark takeoff as complete
        else:
            self.vel.linear.z = 0.0  # Maintain altitude once takeoff is complete
    
    def calculate_error(self, desired_position):
        self.desired_pos.x, self.desired_pos.y, _, _ = desired_position
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
        self.position_error.x, self.position_error.y = abs(self.ex[0]), abs(self.ex[1])
        return self.ex

    def PID(self, kp, ki, kd, error):
        d_ex = np.subtract(error, self.ex0) * 100
        p = np.multiply(error, kp)
        d = np.multiply(d_ex, kd)
        pid = np.add(p, d)
        return pid

    def movement(self, pid):
        self.vel.linear.x = pid[0]
        self.vel.linear.y = pid[1]
        self.vel.linear.z = pid[2]
        self.vel.angular.z = pid[3]
        self.publish_velocities()

class Leader(Drone):
    def __init__(self):
        super(Leader, self).__init__()  # Call super() with no arguments
        self.square_trajectory = generate_square_trajectory()  
        self.current_vertex = 0  
        self.next_vertex = 1 

    def determine_desired_pose(self, number=0):
        # Get the desired position of the current vertex in the trajectory
        if np.all(np.abs(self.ex) < 0.05):  # Small error condition, adjust this value as needed
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
        else:
            pass
        return self.square_trajectory[self.current_vertex]

def generate_square_trajectory():
    square_trajectory = [
        (0, 0, 2, 0),  # First vertex of the square
        (0, 2, 2, 0),  # Second vertex of the square
        (2, 2, 2, 0),  # Third vertex of the square
        (2, 0, 2, 0),  # Fourth vertex of the square
    ]
    return square_trajectory    

def main_function():
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    leader = Leader()

    while not rospy.is_shutdown():
        leader.publish_values()
        leader.take_off()        
        if leader.takeoff_complete:
            
            desired_leader = leader.determine_desired_pose(0)            
            error_leader = leader.calculate_error(desired_leader)
            pl = leader.PID(kp, ki, kd, error_leader)
            leader.movement(pl)

        leader.vel_pub.publish(leader.vel)
        # leader.publish_values()  # Ensure we publish thrust and torque values here
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
