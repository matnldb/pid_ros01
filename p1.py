#!/usr/bin/env python
import numpy as np
import rospy
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

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

kp = [0.25, 0.25, 0.5, 1.5]
kd = [0.05, 0.05, 0.05, 0]
ki = [0, 0 ,0, 0]

class Drone(object):
    def __init__(self):
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()  # Habilita los motores al inicializar el dron
        self.takeoff_complete = False  # Bandera para indicar si el despegue ha sido completado
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)

    def enable_motors(self):
        rospy.wait_for_service('/enable_motors')
        enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            # Si no se ha completado el despegue, ascender gradualmente hasta alcanzar 2 metros
            if self.cord[2] < 2:
                self.vel.linear.z = 0.1
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True  # Marcar el despegue como completado
        else:
            self.vel.linear.z = 0.0  # El despegue ya ha sido completado, mantener la altitud constante
    
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
        # Obtiene la posicin deseada del vrtice actual en la trayectoria
        if np.all(np.abs(self.ex) < 0.05):  # Condicin de error pequeo, ajusta este valor segn sea necesario
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
            rospy.loginfo("Moviendo hacia el vrtice")
        else:
            pass
        return self.square_trajectory[self.current_vertex]

def generate_square_trajectory():
    square_trajectory = [
        (0, 0,2,0),  # Primer vrtice del cuadrado
        (0, 2,2,0),  # Segundo vrtice del cuadrado
        (2, 2,2,0),  # Tercer vrtice del cuadrado
        (2, 0,2,0),  # Cuarto vrtice del cuadrado
    ]
    return square_trajectory    

def main_function():
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    lider = Lider()

    while not rospy.is_shutdown():
        lider.take_off()        
        if lider.takeoff_complete:
            
            deseadas_lider = lider.determine_desired_pose(0)            
            error_lider = lider.calculate_error(deseadas_lider)
            pl = lider.PID(kp,ki,kd,error_lider)
            lider.movement(pl)

        lider.vel_pub.publish(lider.vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
