#!/usr/bin/env python
import numpy as np
import rospy
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
m_ex, m_dex, m_iex, m_pid, m_ex0, vel = Twist(), Twist(), Twist(), Twist(), Twist(), Twist()

#definicion de las constantes para el control en x-y-z-yaw
kp = [0.25, 0.25, 0.5, 1.5]
kd = [0.05, 0.05, 0.05, 0]
ki = [0, 0 ,0, 0]

# posiciones actuales(x,y,z, quaternion_yaw) y deseadas 
cord = [0,0,0,0]
deseadas = [0,0,0,0]

#variables que almacenan el error actual (ex),el error anterior ex0
# la derivada del error y la integral del error
ex = [0,0,0,0]
ex0 = [0,0,0,0]
d_ex = [0,0,0,0]
i_ex = [0,0,0,0]
#senales de salida del controlador PID
pid = [0,0,0,0]
i_saturacion = 500 # saturacion en controlador I
tiempo = 0

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

def refCallback(msg):
     global deseadas
     deseadas[:3] = [msg.x,msg.y,msg.z]

def Saturacion(elemento, saturacion):
    for i in range(4):
        if elemento[i] > saturacion:
            elemento[i] = saturacion
        elif elemento[i] < -saturacion: 
            elemento[i] = -saturacion
    return elemento 

def error():
     global ex, m_ex, ex0, d_ex, i_ex
     ex = np.subtract(deseadas,cord)    # calcula el error
     d_ex = np.subtract(ex,ex0)*100     #calcula la derivada del error
     i_ex = i_ex+np.add(ex, ex0)*0.005  #calcula la integral del error
     ex0 = ex                           # actualiza el error anterior
     i_ex = Saturacion(i_ex, i_saturacion) #limita el error por inetgracion

     m_ex0.linear = actualiza(m_ex0.linear,ex0)
     m_ex0.angular = actualiza(m_ex0.angular,[0,0,ex0[3]]) 
     m_ex.linear = actualiza(m_ex.linear,ex)
     m_ex.angular = actualiza(m_ex.angular,[0,0,ex[3]])
     m_dex.linear = actualiza(m_dex.linear,d_ex)
     m_dex.angular = actualiza(m_dex.angular,[0,0,d_ex[3]]) 
     m_iex.linear = actualiza(m_iex.linear,i_ex)
     m_iex.angular = actualiza(m_iex.angular,[0,0,i_ex[3]]) 

def PID():
    global pid
    p = np.multiply(ex,kp)
    i = np.multiply(i_ex,ki)
    d = np.multiply(d_ex,kd)
    pi = np.add(p,i)
    pid = np.subtract(pi,d)
    m_pid.linear = actualiza(m_pid.linear,pid)
    m_pid.angular = actualiza(m_pid.angular,[0,0,pid[3]]) 

def movement(v): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel.linear = actualiza(vel.linear,v)
        vel.angular = actualiza(vel.angular,[0,0,v[3]])  

def trayectoria():
     global msgs_ref, tiempo
     tiempo +=1
     if tiempo == 1:
       msgs_ref = actualiza(msgs_ref,[0,0,2])
     elif tiempo == 2:
        msgs_ref = actualiza(msgs_ref,[0,2,2])
     elif tiempo == 3:
        msgs_ref = actualiza(msgs_ref,[2,2,2])
     elif tiempo == 3:
        msgs_ref = actualiza(msgs_ref,[2,0,2])
     elif tiempo == 4: 
        msgs_ref = actualiza(msgs_ref,[0,0,2])
     else: msgs_ref = actualiza(msgs_ref,[0,0,0])
     print(tiempo)

        
def main_function():
	global msgs_ref, deseadas, ex, ex0, rate       
	msgs_ref = actualiza(msgs_ref,[0,0,0])	
	counter = 0
	key = ''

	rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	rate = rospy.Rate(50) #Node frequency (Hz)
	    
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #To publish in the topic
	ref_pub = rospy.Publisher("/referencias", Vector3, queue_size=100)
	error_pub = rospy.Publisher("/errores", Twist, queue_size=100)
	derivadas = rospy.Publisher("/derivadas", Twist, queue_size=100);ex0_pub = rospy.Publisher("/ex0", Twist, queue_size=100)
	integrales = rospy.Publisher("/integrales", Twist, queue_size=100);pid_pub = rospy.Publisher("/pid_pub", Twist,queue_size=100)
        
        
	rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
	rospy.Subscriber("/referencias", Vector3, refCallback, queue_size=100)	

	enable_motors() #Execute the functions
	while(key != 'q'):
		error()
		PID()
		velocity_publisher.publish(vel) #Publish the velocities
		ref_pub.publish(msgs_ref);ex0_pub.publish(m_ex0)
		error_pub.publish(m_ex)
		derivadas.publish(m_dex)
		integrales.publish(m_iex);pid_pub.publish(m_pid);movement(pid)#;print(','.join(str(elem) for elem in cord))
		if counter == 50: counter = 0 #Frequency divisor			
		else:counter += 1
		rate.sleep()
		key = getKey()
		if(np.all(np.abs(ex) < 0.001)):trayectoria()
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass