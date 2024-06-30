#!/usr/bin/env python
import numpy as np
import rospy,tf
from nav_msgs.msg import Path
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import WrenchStamped


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
kpCol = [0.5, 0.5, 0, 0]

class Drone(object):
    def __init__(self, name):
        self.name = name
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.angls = [0,0]
        self.desired_pos = Point()
        self.position_error = Point()

        self.vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher(self.name +'/position', Point, queue_size=10)
        self.att_pub = rospy.Publisher(self.name +'/angles', Point, queue_size=10)
        self.desired_pose_pub = rospy.Publisher(self.name +'/desired_position', Point, queue_size=10)
        self.error_pub = rospy.Publisher(self.name +'/position_error', Point, queue_size=10)
        
        # Publicadores para las velocidades
        self.vel_linear_x_pub = rospy.Publisher(self.name +'/cmd_vel/linear/x', Float32, queue_size=10)
        self.vel_linear_y_pub = rospy.Publisher(self.name +'/cmd_vel/linear/y', Float32, queue_size=10)
        self.vel_angular_z_pub = rospy.Publisher(self.name +'/cmd_vel/angular/z', Float32, queue_size=10)

        # Publishers for visualization in rqt_plot
        self.thrust_pub = rospy.Publisher(self.name +'/visualize/thrust', Float32, queue_size=10)
        self.torque_x_pub = rospy.Publisher(self.name +'/visualize/torque_x', Float32, queue_size=10)
        self.torque_y_pub = rospy.Publisher(self.name +'/visualize/torque_y', Float32, queue_size=10)
        self.torque_z_pub = rospy.Publisher(self.name +'/visualize/torque_z', Float32, queue_size=10)

        # Subscribe to the /command/wrench topic
        rospy.Subscriber('/command/wrench', WrenchStamped, self.wrench_callback)
                
        self.pose_sub = rospy.Subscriber(self.name + '/ground_truth/state', Odometry, self.poseCallback)
        # self.path_pub = rospy.Publisher(self.name + '/path', Path, queue_size=10)
        self.path = Path()
        # self.path.header.frame_id = "world" 
        self.enable_motors()  # Habilita los motores al inicializar el dron
        self.takeoff_complete = False  # Bandera para indicar si el despegue ha sido completado
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
        (self.angls[0], self.angls[1], self.cord[3]) = euler_from_quaternion(quater_list)
        self.publish_position()        


        # Incluye la pose actual al path
        # drone_pose = PoseStamped()       
        # drone_pose.pose.position = msg.pose.pose.position
        # self.path.poses.append(drone_pose)
        # self.path.header.stamp = drone_pose.header.stamp
        # self.path_pub.publish(self.path) 

        # br = tf.TransformBroadcaster()
        # br.sendTransform( (self.cord[:3]), 
		# tf.transformations.quaternion_from_euler(y, p, self.cord[3]), rospy.Time.now(), 
		# self.name + "/base_link", "world" )      

    def publish_position(self):
        point = Point(x=self.cord[0], y=self.cord[1], z = self.cord[2])
        self.pose_pub.publish(point)
        rpy = Point(x=self.angls[0], y=self.angls[0], z = self.cord[3])
        self.att_pub.publish(rpy)
        self.desired_pose_pub.publish(Point(x=self.desired_pos.x, y=self.desired_pos.y))
        self.error_pub.publish(Point(x=self.position_error.x, y=self.position_error.y))

    def publish_velocities(self):
        # Publicar las velocidades actuales
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
        rospy.wait_for_service(self.name + '/enable_motors')
        enable_motors = rospy.ServiceProxy(self.name + '/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            # Si no se ha completado el despegue, ascender gradualmente hasta alcanzar 2 metros
            if self.cord[2] < 2:
                self.vel.linear.z = 0.2
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True  # Marcar el despegue como completado
        else:
            self.vel.linear.z = 0.0  # El despegue ya ha sido completado, mantener la altitud constante
    
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

class Lider(Drone):
    def __init__(self, name):
        super(Lider, self).__init__(name)  # Llama a super() sin argumentos
        self.square_trajectory = generate_square_trajectory()  
        self.current_vertex = 0  
        self.next_vertex = 1 

    def determine_desired_pose(self, number=0):
        # Obtiene la posicin deseada del vrtice actual en la trayectoria
        if np.all(np.abs(self.ex) < 0.05):  # Condicin de error pequeo, ajusta este valor segn sea necesario
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
    def __init__(self, name, Lider, Vecino):
        super(Seguidor, self).__init__(name)
        self.lider = Lider.name
        self.vecino = Vecino
        self.leader_position = [0, 0, 0, 0]
        self.neighbor_position = [0, 0, 0, 0]
        rospy.Subscriber(self.lider+'/ground_truth/state', Odometry, self.leaderPoseCallback)
        rospy.Subscriber(self.vecino+'/ground_truth/state', Odometry, self.neighborPoseCallback)
        self.exCol = [0,0,0,0]
        self.ex0Col = [0,0,0,0] 
        self.distance_pub = rospy.Publisher(self.name + '/distance_to_leader', Float32, queue_size=10)
        self.distError_pub = rospy.Publisher(self.name + '/error_distancia', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher(self.name + '/angle_to_leader', Float32, queue_size=10)
        self.anglError_pub = rospy.Publisher(self.name + '/error_angle', Float32, queue_size=10)
        self.distNeighbor_pub = rospy.Publisher(self.name +'/distVecino', Float32, queue_size=10)


    def leaderPoseCallback(self, msg):
        pos = msg.pose.pose.position
        self.leader_position[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.leader_position[3]) = euler_from_quaternion(quater_list)

    def neighborPoseCallback(self, msg):
        pos = msg.pose.pose.position
        self.neighbor_position[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.neighbor_position[3]) = euler_from_quaternion(quater_list)
        

    def determine_desired_pose(self, desired_angle):
        # En el caso del seguidor, la posicin deseada depende de la posicin del lder
        distance_guard = np.sqrt(2)
        corDiff = np.subtract((self.cord[:2]),(self.neighbor_position[:2]))
        distance_to_neighbor = np.linalg.norm(corDiff)                
        if distance_to_neighbor <= distance_guard:
             aux = (distance_guard - distance_to_neighbor) / distance_to_neighbor
             correction = np.multiply(corDiff,aux+1)
             newPose =  np.add(self.cord[:2],correction)
             return np.hstack((newPose, self.cord[2:4]))
        else: 
            desired_distance = 2
            x, y, z, yaw = self.leader_position  # Utilizar la posicin del ler actualizada
            desired_x = x + desired_distance * np.cos(np.radians(desired_angle))
            desired_y = y + desired_distance * np.sin(np.radians(desired_angle))
            return [desired_x, desired_y, z, yaw]  # Por ejemplo, mantener la misma altitud que el lder
        
    def errorControler(self, desired_angle=0):
        x1, y1, z1, yaw1 = self.leader_position
        x2, y2, z2, yaw2 = self.cord
        x3,y3,z3, yaw3 = self.neighbor_position

        delta_y = y1 - y2
        delta_x = x1 - x2
    
        if delta_x == 0:
            angle_radians = np.pi / 2 if delta_y > 0 else -np.pi / 2
        else:
            angle_radians = np.arctan2(delta_y, delta_x)

        angle_degrees = np.degrees(angle_radians)

        vectorA = np.array([x2 - x1, y2 - y1])
        vectorB = np.array([x3 - x2, y3 - y2])

        normA = np.linalg.norm(vectorA)
        normB = np.linalg.norm(vectorB)

        self.distance_pub.publish(normA)
        self.angle_pub.publish(-1*angle_degrees)

        self.distNeighbor_pub.publish(normB)

        self.distError_pub.publish(2.8-normB)
        self.anglError_pub.publish(np.radians(desired_angle)-angle_radians)

def main_function():
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    lider = Lider("uav1")
    seguidor1 = Seguidor("uav2",lider,"uav3" )
    seguidor2 = Seguidor("uav3",lider,"uav2")    


    while not rospy.is_shutdown():
        lider.publish_values()
        seguidor1.publish_values
        seguidor2.publish_values()       
        lider.take_off()
        seguidor1.take_off()
        seguidor2.take_off()

        seguidor1.errorControler(135)
        seguidor2.errorControler(45)


        if lider.takeoff_complete and seguidor1.takeoff_complete:
            
            deseadas_lider = lider.determine_desired_pose(0)
            deseadas_seguidor = seguidor1.determine_desired_pose(135)
            deseadas_seguidor2= seguidor2.determine_desired_pose(45)
            
            error_lider = lider.calculate_error(deseadas_lider)
            error_seguidor1 = seguidor1.calculate_error(deseadas_seguidor)
            error_seguidor2 = seguidor2.calculate_error(deseadas_seguidor2) 

            pl = lider.PID(kp,ki,kd,error_lider)
            ps1 = seguidor1.PID(kp, ki,kd,error_seguidor1)
            ps2 = seguidor2.PID(kp, ki,kd,error_seguidor2)
            lider.movement(pl)
            seguidor1.movement(ps1)
            seguidor2.movement(ps2)


        lider.vel_pub.publish(lider.vel)
        # lider.path_pub.publish(lider.path)
        seguidor1.vel_pub.publish(seguidor1.vel)
        # seguidor1.path_pub.publish(seguidor1.path) 
        seguidor2.vel_pub.publish(seguidor2.vel)
        # seguidor2.path_pub.publish(seguidor2.path)        


        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
