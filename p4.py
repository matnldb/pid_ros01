#!/usr/bin/env python
import numpy as np
import rospy
import pyswarm as ps
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Quaternion
from tf.transformations import euler_from_quaternion

#######################################################################################
#kbhit function implemented on Linux
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
#######################################################################################

# msg_takeoff, msg_land = Empty(),Empty()
msgs_ref = Vector3()
vel = Twist()
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# posiciones actuales(x,y,z, quaternion_yaw) y deseadas 
cord = [0,0,0,0]; i_ex = [0,0,0,0] 
fin = True
tiempo = 0
count = 0

def actualiza(v,valor):
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v

def poseCallback(msg): #Callback function to get the drone posture
        global cord
        pos = msg.pose.pose.position
        cord[:3] = [pos.x, pos.y, pos.z]       
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r,p,cord[3]) = euler_from_quaternion(quater_list) #Euler angles are given in radians       


def Saturacion(elemento, saturacion):
    for i in range(4):
        if elemento[i] > saturacion:
            elemento[i] = saturacion
        elif elemento[i] < -saturacion: 
            elemento[i] = -saturacion
    return elemento 
  
def PID(kp,ki,kd, ex,ex0):
    global i_ex
    d_ex = np.subtract(ex,ex0)*100
    i_ex = i_ex+np.add(ex, ex0)*0.005
    ex0 = ex
    i_ex = Saturacion(i_ex, 500)
    p = np.multiply(ex,kp)
    i = np.multiply(i_ex,ki)
    d = np.multiply(d_ex,kd)
    pi = np.add(p,i)
    pid = np.subtract(pi,d)
    return pid

def movement(v): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel.linear = actualiza(vel.linear,v)
        vel.angular = actualiza(vel.angular,[0,0,v[3]])  

def trayectoria():
     global tiempo,fin,i_ex
     i_ex = [0,0,0,0]
     tiempo +=1
     #print(tiempo)
     if tiempo == 0:
       return [0,0,0,0]
     elif tiempo == 1:
       return [0,0,2,0]
     elif tiempo == 2:
        return [0,2,2,0]
     elif tiempo == 3:
        return [2,2,2,0]
     elif tiempo == 4:
        return [2,0,2,0]
     elif tiempo == 5: 
        return [0,0,2,0]
     elif tiempo == 6: 
        return [0,0,0,0]
     else: 
         fin = False
         tiempo = 0
         return [0,0,0,0]     

def cambio(ex):
    global count
    if(np.all(np.abs(ex) < 0.09)):
        return True
    else: return False

def objective_function(x):
    
    global i_ex, velocity_publisher,fin, tiempo
    # Coeficientes a optimizar
    kp = x[:4]
    ki = x[4:8]
    kd = x[8:12]
    fin = True
    # tiempo = 0  
    i_ex = [0,0,0,0]
    J0 = 0 #error anterior
    J = 0#integral del error anterior
    deseadas = [0,0,0,0]
    ex = [0,0,0,0]
    print("kp: ",kp)
    print("ki: ",ki)
    print("kd: ",kd)
    print("**********************")
    index = 0
    # Restaurar los valores iniciales de las variables
    while(fin):
            ex0 = ex
            # print("Model ", index, ex)
            index += 1
            if(cambio(ex)):
                deseadas = trayectoria()
            ex = np.subtract(deseadas,cord)		
            movement(PID(kp,ki,kd,ex,ex0))
            velocity_publisher.publish(vel)
            vector_traspuesto = ex.reshape(-1, 1)
            #print(vector_traspuesto)
            escalar = np.dot(ex,vector_traspuesto)[0]
            J+= (escalar+J0)*0.005
            #print("camino",index, "J: ", J)            
            J0 = escalar           
            rate.sleep()
    print(J)
    return J     	
        
def main_function():
        global rate	
        rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
        rate = rospy.Rate(50) #Node frequency (Hz
        rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
        inferior = [0.2, 0.2, 0.4, 1.4,0.0, 0.0, 0.0, 0,0, 0 ,0, 0]
        superior = [0.3, 0.3, 0.6, 1.6,0.05, 0.05, 0.05, 0.1,0.1, 0.1 ,0.1, 0.1]
        enable_motors() #Execute the functions
        xopt, fopt = ps.pso(objective_function, inferior, superior)
        # Imprimir los resultados
        print("Los mejores coeficientes encontrados:")
        print("kp =", xopt[:4])
        print("kd =", xopt[4:8])
        print("ki =", xopt[8:12])
        print("Valor mnimo del error medio cuadrtico:", fopt)         		
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass