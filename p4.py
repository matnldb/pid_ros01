#!/usr/bin/env python
import numpy as np
import rospy
import time
import pyswarm as ps
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

def getKey():
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

cambio = False

class Drone(object):
    def __init__(self):
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"
        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()
        self.takeoff_complete = False
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]
        self.count = 0
        self.mse = 0
        self.band = 0
        self.goDown = False

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)

        # Update path for visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = msg.pose.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

    def enable_motors(self):
        rospy.wait_for_service('/enable_motors')
        enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            if self.cord[2] < 2:
                self.vel.linear.z = 0.2
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True
        else:
            self.vel.linear.z = 0.0

    def land(self):
        if not self.goDown:
            if self.cord[2] > 0.3:
                self.vel.linear.z = -0.2
            else:
                self.vel.linear.z = 0.0
                self.goDown = True
        else:
            self.vel.linear.z = 0.0

    def calculate_error(self, desired_position):
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
        return self.ex

    def calcula_MSE(self, error):
        self.mse += np.sum(np.square(error))
        self.count += 1

    def reset(self):
        global cambio
        cambio = False
        self.mse = 0
        self.count = 0

    def PID(self, kp, kd, error):
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
        super(Lider, self).__init__()
        self.square_trajectory = generate_square_trajectory()
        self.current_vertex = -1
        self.next_vertex = 0
        self.MSE2 = 0
        self.cuenta = 0

    def determine_desired_pose(self, desired_angle=0):
        global cambio
        if np.all(np.abs(self.ex) < 0.25):
            if self.band != 0:
                cambio = True
            self.band +=1
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
        else:
            pass
        return self.square_trajectory[self.current_vertex]

def generate_square_trajectory():
    return [
        (0, 0, 2, 0),
        (0, 2, 2, 0),
        (2, 2, 2, 0),
        (2, 0, 2, 0)
    ]
def funcionObjetivo(gains):
    global cambio
    kp = gains[:4]
    kd = gains[4:]
    print("kp: ", kp)
    print("kd: ", kd)
    inicio = time.time()
    lado = 0
    MSE2 = 0    
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)
    lider = Lider()
    while not rospy.is_shutdown():
        lider.take_off()        
        if lider.takeoff_complete:
            
            deseadas_lider = lider.determine_desired_pose(0)
            error_lider = lider.calculate_error(deseadas_lider)
            lider.calcula_MSE(error_lider)
            pl = lider.PID(kp,kd,error_lider)
            lider.movement(pl)
            if lado < 4: 
                if cambio:
                    MSE2 += lider.mse/lider.count
                    lado+=1
                    lider.reset()
            else:
                lider.land()
                if lider.goDown:
                   final = time.time()
                   print("MSE: ", MSE2/4)
                   print("tiempo de convergencia", final-inicio)
                   return MSE2/4         
        lider.vel_pub.publish(lider.vel)        
        rate.sleep()
    return lider.MSE2/4

def main_function():
    inferior = [0, 0, 0, 0, 0, 0, 0, 0]
    superior = [2, 2, 2, 2, 2, 2, 2, 2]
    xopt, fopt = ps.pso(funcionObjetivo, inferior, superior, swarmsize=5, maxiter=3)
    print("Los mejores coeficientes encontrados:")
    print("kp =", xopt[:4])
    print("kd =", xopt[4:8])
    print("Valor mnimo del error medio cuadrtico:", fopt)

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
