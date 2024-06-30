#!/usr/bin/env python
import numpy as np
import rospy
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

import sys, select, os # Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

def getKey(): # Function to use keyboard events on Linux
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
        self.cord = [0, 0, 0, 0]  # Coordinates [x, y, z, yaw]
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.poseCallback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)  # Actual path publisher
        self.tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10)  # Desired path publisher
        self.actual_path = Path()
        self.desired_path = Path()
        self.actual_path.header.frame_id = "world"  # Set the frame id to world
        self.desired_path.header.frame_id = "world"
        self.enable_motors()  # Enable motors upon initializing the drone
        self.takeoff_complete = False  # Flag to indicate if takeoff has been completed
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)
        # Append current drone position to the actual path
        pose_stamped = PoseStamped()
        pose_stamped.pose = msg.pose.pose
        pose_stamped.header.stamp = rospy.Time.now()
        self.actual_path.poses.append(pose_stamped)
        self.path_pub.publish(self.actual_path)  # Publish the actual path

    def enable_motors(self):
        rospy.wait_for_service('/enable_motors')
        enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            if self.cord[2] < 2:
                self.vel.linear.z = 0.1
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True
        else:
            self.vel.linear.z = 0.0

    def calculate_error(self, desired_position):
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
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

class Lider(Drone):
    def __init__(self):
        super(Lider, self).__init__()  # Llama a super() sin argumentos
        self.square_trajectory = generate_square_trajectory()  
        self.current_vertex = 0  
        self.next_vertex = 1 

    def determine_desired_pose(self, number=0):
        if np.all(np.abs(self.ex) < 0.05):
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
        desired_pose = self.square_trajectory[self.current_vertex]
        # Append desired pose to the desired path
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = desired_pose[:3]
        pose_stamped.header.stamp = rospy.Time.now()
        self.desired_path.poses.append(pose_stamped)
        self.tr_path_pub.publish(self.desired_path)  # Publish the desired path
        return desired_pose

def generate_square_trajectory():
    return [
        (0, 0, 2, 0),
        (0, 2, 2, 0),
        (2, 2, 2, 0),
        (2, 0, 2, 0)
    ]

def main_function():
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)
    lider = Lider()

    while not rospy.is_shutdown():
        lider.take_off()
        if lider.takeoff_complete:
            desired_lider = lider.determine_desired_pose(0)
            error_lider = lider.calculate_error(desired_lider)
            pl = lider.PID(kp, ki, kd, error_lider)
            lider.movement(pl)

        lider.vel_pub.publish(lider.vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
