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
msgs_ref, msgs_rotacion  = Vector3(), Vector3()
m_ex, m_dex, m_iex = Twist(), Twist(), Twist()


#Define global variables (position and orientation, using Euler anlges)
motor_off=[0,0,0,0] #x-y-z-yaw
takeoff_alt = 1.2 #Desired initial altitude
despegue = [0,0,0.2,0]
aterrizar = [0,0,-0.2,0]
circular = [0.4,0,0,0.5]
#variables para limitar el espacio de movimiento del drone 
restriccion = 1.20
restriccionz = 2

#definicion de las constantes para el control en x-y-z-yaw
kp = [0.25, 0.25, 0.5, 1.5]
kd = [0.05, 0.05, 0.05, 0]
ki = [0, 0 ,0, 0]

# posiciones actuales(x,y,z, quaternion_yaw) y deseadas 
cord = [0,0,0,0]
deseadas = [0,0,0,0]
dYawd = 0 

#variables que almacenan el error actual (ex),el error anterior ex0
# la derivada del error y la integral del error
ex = [0,0,0,0]
ex0 = [0,0,0,0]
d_ex = [0,0,0,0]
i_ex = [0,0,0,0]

#senales de salida de los controladores P, I, D, PD, PID
control_out = [0,0,0,0]
i_saturacion = 500 # saturacion en controlador I
velineal = [0,0,0]  #velocidades lineales y angulos RPY
rpy_ang = [0,0,0]

vel = Twist() #Create Twist message instance

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
     global ex, m_ex       
     ex = np.subtract(deseadas,cord)
     m_ex.linear = actualiza(m_ex.linear,ex)
     m_ex.angular = actualiza(m_ex.angular,[0,0,ex[3]])
     
def dError():
     global d_ex, m_dex
     d_ex = np.subtract(ex,ex0)*100
     m_dex.linear = actualiza(m_dex.linear,ex)
     m_dex.angular = actualiza(m_dex.angular,[0,0,d_ex[3]])
     
def iError():
     global i_ex, m_iex
     i_ex = i_ex+np.add(ex, ex0)*0.005
     i_ex = Saturacion(i_ex, i_saturacion)
     m_iex.linear = actualiza(m_iex.linear,ex)
     m_iex.angular = actualiza(m_iex.angular,[0,0,i_ex[3]])
     
def movement(v): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel.linear.x = v[0]
        vel.linear.y = v[1]
        vel.linear.z = v[2]
        vel.angular.z = v[3]        

def takeoff(): #Takeoff function
	movement(despegue)
	rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)
	while(cord[2] < takeoff_alt-0.1):
		velocity_publisher.publish(vel)
		ref_pub.publish(msgs_ref)
		rate.sleep(); 
		key = getKey()
		if(key == 'q'):
			break	
	movement(motor_off)
	velocity_publisher.publish(vel)
	ref_pub.publish(msgs_ref)

def land(): #Land function
	movement(aterrizar)
	velocity_publisher.publish(vel)
	ref_pub.publish(msgs_ref)
	print("\n Landing ...\n")
	
	while(cord[2] > 0.3):
		velocity_publisher.publish(vel)
		rate.sleep(); 	
	movement(motor_off)
	velocity_publisher.publish(vel)	
	 

def main_function():
	global velocity_publisher, msgs_ref, deseadas, ex
	global ref_pub, error_pub, derivadas, integrales
       
	xd = float(input("intro xd: "))
	yd = float(input("intro yd: "))
	zd = float(input("intro zd: "))
	msgs_ref = actualiza(msgs_ref,[xd,yd,zd])
	error()
	dError()
	iError()

	rospy.init_node("hector_quadrotor_simple_movement", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
    
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #To publish in the topic
	ref_pub = rospy.Publisher("/referencias", Vector3, queue_size=100)
	error_pub = rospy.Publisher("/errores", Twist, queue_size=100)
	derivadas = rospy.Publisher("/derivadas", Twist, queue_size=100)
	integrales = rospy.Publisher("/integrales", Twist, queue_size=100)    
        
	rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
	rospy.Subscriber("/referencias", Vector3, refCallback, queue_size=100)
	

	enable_motors() #Execute the functions
	takeoff()
	movement(circular)	#Circular movement on the horizontal plane
	
	while(1):
		error()
		dError()
		iError()
		velocity_publisher.publish(vel) #Publish the velocities
		ref_pub.publish(msgs_ref)
		error_pub.publish(m_ex)
		derivadas.publish(m_dex)
		integrales.publish(m_iex)
		
		#print(','.join(str(elem) for elem in cord))
		if counter == 50: counter = 0 #Frequency divisor			
		else:counter += 1
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'q'):break
	land() #Execute land function
        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass