#!/usr/bin/env python
import numpy as np
import rospy
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from pyswarm import pso

class Drone(object):
    def __init__(self, kp, kd):
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()  # Habilita los motores al inicializar el dron
        self.takeoff_complete = False  # Bandera para indicar si el despegue ha sido completado
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]
        self.kp = kp
        self.kd = kd
        self.total_mse = 0

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
                self.vel.linear.z = 0.25
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True  # Marcar el despegue como completado
        else:
            self.vel.linear.z = 0.0  # El despegue ya ha sido completado, mantener la altitud constante
    
    def calculate_error(self, desired_position):
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
        return self.ex

    def PID(self, error):
        d_ex = np.subtract(error, self.ex0) * 100
        p = np.multiply(error, self.kp)
        d = np.multiply(d_ex, self.kd)
        pid = np.add(p, d)
        return pid

    def movement(self, pid):
        self.vel.linear.x = pid[0]
        self.vel.linear.y = pid[1]
        self.vel.linear.z = pid[2]
        self.vel.angular.z = pid[3]
        self.vel_pub.publish(self.vel)

    def reset_mse(self):
        self.total_mse = 0

class Lider(Drone):
    def __init__(self):
        super(Lider, self).__init__()  # Llama a super() sin argumentos
        self.square_trajectory = generate_square_trajectory()  
        self.current_vertex = 0  
        self.next_vertex = 1
        self.mse_list = []

    def determine_desired_pose(self, number=0):
        # Obtiene la posición deseada del vértice actual en la trayectoria        
        if np.all(np.abs(self.ex) < 0.05):  # Condición de error pequeño, ajusta este valor según sea necesario
            if self.current_vertex == 0:
                # Si es el primer vértice, resetea el MSE total
                self.reset_mse()

            if self.current_vertex > 0:
                # Si no es el primer vértice, calcula el MSE del tramo y agrégalo al MSE total
                mse = np.sum(np.square(self.ex))
                self.total_mse += mse
                self.mse_list.append(mse)

            if self.current_vertex == len(self.square_trajectory) - 1:
                # Si es el último vértice, calcula el MSE promedio de toda la trayectoria
                average_mse = self.total_mse / len(self.mse_list)
                print("MSE promedio de toda la trayectoria:", average_mse)

            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)

        return self.square_trajectory[self.current_vertex]

def generate_square_trajectory():
    square_trajectory = [
        (0, 0, 2, 0),  # Primer vértice del cuadrado
        (0, 2, 2, 0),  # Segundo vértice del cuadrado
        (2, 2, 2, 0),  # Tercer vértice del cuadrado
        (2, 0, 2, 0),  # Cuarto vértice del cuadrado
    ]
    return square_trajectory    

def objective_function(gains):
    # Función objetivo para evaluar el desempeño del controlador PID con las ganancias dadas
    kp = gains[:4]
    kd = gains[4:]
    drone = Lider()
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        drone.take_off()        
        if drone.takeoff_complete:
            deseadas_lider = drone.determine_desired_pose(0)            
            error_lider = drone.calculate_error(deseadas_lider)
            pl = drone.PID(error_lider)
            drone.movement(pl)
        rate.sleep()
    # Devuelve el MSE promedio de toda la trayectoria
    return drone.total_mse

def main():
    # Define el rango de búsqueda para las ganancias del controlador
    lower_bound = [0, 0, 0, 0, 0, 0, 0, 0]
    upper_bound = [10, 10, 10, 10, 10, 10, 10, 10]

    # Ejecuta PSO para encontrar las mejores ganancias del controlador
    best_gains, _ = pso(objective_function, lower_bound, upper_bound)

    print("Mejores ganancias encontradas:", best_gains)

if __name__ == '__main__':
    main()
