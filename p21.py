#!/usr/bin/env python
import numpy as np
import rospy
import pyswarm as ps
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
cambio = False

class Drone(object):
    def __init__(self, name):
        self.name = name
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.name + '/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()  # Habilita los motores al inicializar el dron
        self.takeoff_complete = False  # Bandera para indicar si el despegue ha sido completado
        self.goDown = False
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]
        self.count = 0
        self.mse = 0
        self.band = 0

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)

    def enable_motors(self):
        rospy.wait_for_service(self.name + '/enable_motors')
        enable_motors = rospy.ServiceProxy(self.name + '/enable_motors', EnableMotors)
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
    
    def land(self):
        if not self.goDown:
            if self.cord[2] > 0.3:
                self.vel.linear.z = -0.2  # Descender gradualmente
            else:
                self.vel.linear.z = 0.0
                self.goDown = True  # El aterrizaje ha sido completado
        else: 
            self.vel.linear.z = 0.0
    
    def calculate_error(self, desired_position):
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
        return self.ex
    
    def calcula_MSE(self, error):
        self.mse += np.sum(np.square(error))
        self.count +=1

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
    def __init__(self, name):
        super(Lider, self).__init__(name)  # Llama a super() sin argumentos
        self.square_trajectory = generate_square_trajectory()  
        self.current_vertex = 0  
        self.next_vertex = 1 

    def determine_desired_pose(self, desired_angle=0):
        global cambio
        # Obtiene la posicin deseada del vrtice actual en la trayectoria
        if np.all(np.abs(self.ex) < 0.05):  # Condicin de error pequeo, ajusta este valor segn sea necesario
            if self.band != 0:
                cambio = True
            self.band +=1
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
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

class Seguidor(Drone):
    def __init__(self, name, Lider):
        super(Seguidor, self).__init__(name)
        self.lider = Lider.name
        self.leader_position = [0, 0, 0, 0]
        rospy.Subscriber(self.lider+'/ground_truth/state', Odometry, self.leaderPoseCallback)
        self.exCol = [0,0,0,0]

    def leaderPoseCallback(self, msg):
        pos = msg.pose.pose.position
        self.leader_position[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.leader_position[3]) = euler_from_quaternion(quater_list)

    def determine_desired_pose(self, desired_angle):        
        desired_distance = 2
        x, y, z, yaw = self.leader_position  # Utilizar la posicin del ler actualizada
        desired_x = x + desired_distance * np.cos(np.radians(desired_angle))
        desired_y = y + desired_distance * np.sin(np.radians(desired_angle))
        return [desired_x, desired_y, z-0.4, yaw]  

def funcObjetivo(gains):
    
    global cambio, kp, kd
    kpg = gains[:4]
    kdg = gains[4:]
    print("kp: ", kpg)
    print("kd: ", kdg)
    lado = 0
    MSE2 = 0    
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    lider = Lider("uav1")
    seguidor1 = Seguidor("uav2",lider)
    while not rospy.is_shutdown():
        
        lider.take_off()
        seguidor1.take_off()

        if lider.takeoff_complete and seguidor1.takeoff_complete:
            
            deseadas_lider = lider.determine_desired_pose(0)
            deseadas_seguidor = seguidor1.determine_desired_pose(45)
            
            error_lider = lider.calculate_error(deseadas_lider)
            error_seguidor1 = seguidor1.calculate_error(deseadas_seguidor)

            seguidor1.calcula_MSE(error_seguidor1)

            pl = lider.PID(kp,kd,error_lider)
            ps1 = seguidor1.PID(kpg,kdg,error_seguidor1)

            lider.movement(pl)
            seguidor1.movement(ps1)

            if lado < 4: 
                if cambio:
                    MSE2 += seguidor1.mse/seguidor1.count
                    lado+=1
                    seguidor1.reset()
            else:
                lider.land()
                seguidor1.land()
                if lider.goDown and seguidor1.goDown:
                   print("MSE: ", MSE2/4)
                   return MSE2/4 


        lider.vel_pub.publish(lider.vel)
        seguidor1.vel_pub.publish(seguidor1.vel)
        rate.sleep()
    return MSE2/4

def main_function():
    inferior = [0, 0, 0, 0, 0, 0, 0, 0]
    superior = [2, 2, 2, 2, 2, 2, 2, 2]  
    
    xopt, fopt = ps.pso(funcObjetivo, inferior, superior) #swarmsize=5, maxiter=3
    # Imprimir los resultados
    print("Los mejores coeficientes encontrados:")
    print("kp =", xopt[:4])
    print("kd =", xopt[4:8])
    print("Valor mnimo del error medio cuadrtico:", fopt)    

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
