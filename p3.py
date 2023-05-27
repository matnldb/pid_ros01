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
cord = [0,0,0,0]; i_ex = [0,0,0,0]; ex0 = [0,0,0,0]
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
  
def PID(kp,ki,kd, ex):
    global pid, ex0,i_ex
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
     global tiempo
     tiempo +=1
     print(tiempo)
     if tiempo == 1:
       return [0,0,2,0]
     elif tiempo == 2:
        return [0,2,2,0]
     elif tiempo == 3:
        return [2,2,2,0]
     elif tiempo == 3:
        return [2,0,2,0]
     elif tiempo == 4: 
        return [0,0,2,0]
     else: 
         #tiempo=0;
         return [0,0,0,0]     

def cambio(ex):
    global count
    if(np.all(np.abs(ex) < 0.05)):
        count+=1
    else: count = 0 
    if(count == 10):return True
    else: return False

def objective_function(x):
    
    global i_ex, velocity_publisher
    # Coeficientes a optimizar
    kp = x[:4]
    kd = x[4:8]
    ki = x[8:12]

    ac2 = [0,0,0,0]

    # Restaurar los valores iniciales de las variables
    if(cambio(ex)):
        deseadas = trayectoria()
    ex = np.subtract(deseadas,cord)
    movement(PID(kp,ki,kd,ex))
    velocity_publisher.publish(vel) 
    rate.sleep()    	
        
def main_function():
	global msgs_ref, ex, ex0, rate       
	msgs_ref = actualiza(msgs_ref,[0,0,0])	
	ex = [0,0,0,0]; kp = [0.25, 0.25, 0.5, 1.5]; kd = [0.05, 0.05, 0.05, 0]; ki = [0, 0 ,0, 0]
	deseadas = [0,0,0,0]
	rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	rate = rospy.Rate(50) #Node frequency (Hz)
	    
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)         
	rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
	
	enable_motors() #Execute the functions
	while(getKey() != 'q'):
            if(cambio(ex)):
                deseadas = trayectoria()
            ex = np.subtract(deseadas,cord)		
            movement(PID(kp,ki,kd,ex))
            velocity_publisher.publish(vel) 
            rate.sleep()            		
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass