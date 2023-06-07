#!/usr/bin/env python
import numpy as np
import rospy
import pyswarm as ps
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Vector3, PoseStamped
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

msgs_ref = Vector3()
vel = Twist()
drone_path_msg = Path()
tr_path_msg = Path()
global_frame = "/world"
# posiciones actuales(x,y,z, quaternion_yaw) y deseadas 
cord = [0,0,0,0]; i_ex = [0,0,0,0]; 
tiempo = 0
count = 0
t0 = 0.0
t = 0.0
J0 = 0 #error anterior
J = 0
tx = 0
cordx = [0,0,0]
fin = True
velocidades = []

def actualiza(v,valor):
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v

def poseCallback(msg): #Callback function to get the drone posture
        global cord, drone_path_msg
        pos = msg.pose.pose.position
        cord[:3] = [pos.x, pos.y, pos.z]       
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r,p,cord[3]) = euler_from_quaternion(quater_list) #Euler angles are given in radians       
        velocidad()
        drone_pose = PoseStamped() #Create the PoseStamped instance
        drone_pose.pose.position = msg.pose.pose.position
        drone_path_msg.poses.append(drone_pose) #Add the drone position to the list of points

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
     global tiempo,fin,tr_path_msg,i_ex,t,t0
     tiempo +=1
     i_ex = [0,0,0,0]    
     t1 = rospy.Time.now().to_sec() - t0
     t0 = t1
     t += t1
     #if tiempo <6: print(t1)
     if tiempo == 1: 
       print("--:tiempo 1",t1)  
       d = [0,0,2,0]
     elif tiempo == 2:
        print("--:tiempo 2",t1) 
        d =  [0,2,2,0]
     elif tiempo == 3:
       print("--:tiempo 3",t1) 
       d =  [2,2,2,0]
     elif tiempo == 4:
        print("--:tiempo 4",t1) 
        d =  [2,0,2,0]
     elif tiempo == 5: 
        print("--:tiempo 5",t1) 
        d =  [0,0,2,0]
     elif tiempo == 6: 
        print("--:tiempo 6",t1) 
        d =  [0,0,0,0]
     else: 
         print("--:tiempo total")         
         print(t) 
         print("la velocidad mayor fue")
         print(max(velocidades))                 
         fin = False
         d = [0,0,0,0]
     print("El error cuadratico medio se calculo: ")
     print(J)       
     t1 = 0
     tr_pose = PoseStamped() #Create the PoseStamped instance
     tr_pose.pose.position = actualiza(tr_pose.pose.position,d)     
     tr_path_msg.poses.append(tr_pose) #Add the new point to the list
     return d     

def velocidad():
    global tx, cordx, velocidades
    tx2 = rospy.Time.now().to_sec() 
    dtx = tx2 - tx
    tx = tx2
    a = np.array(cord[:3])
    b = np.array(cordx)
    d = np.linalg.norm(a - b)    
    cordx = cord[:3]
    if dtx!= 0:
        v = d/dtx
        velocidades.append(v)    

def funcionCosto(ex):
    global J, J0   
    vector_traspuesto = ex.reshape(-1, 1)
    escalar = np.dot(ex,vector_traspuesto)[0]
    J+= (escalar+J0)*0.005            
    J0 = escalar
    return J

def cambio(ex):
    global count
    if(np.all(np.abs(ex) < 0.09)):
        return True
    else: return False
        
def main_function():
        global msgs_ref, rate,fin,t0,tx   
        
        ex = [0,0,0,0]; 
        #kp = [0.25, 0.25, 0.5, 1.5]; kd = [0.05, 0.05, 0.05, 0]; ki = [0, 0 ,0, 0]
        kp = [ 0.29060169,  0.22351094,  0.54439942,  1.53864866]
        ki = [ 0.0493674 ,  0.05      ,  0.0378391 ,  0.00537379]
        kd = [ 0.01099189,  0.06127233,  0.0423897 ,  0.04541789] 
        deseadas = [0,0,0,0]
        rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
        rate = rospy.Rate(50) #Node frequency (Hz)
        tx = rospy.Time.now().to_sec()
            
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)         
        rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
        
        #To publish the paths
        drone_path_pub = rospy.Publisher('/path', Path, queue_size=10)
        tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10) 
        
        global drone_path_msg, tr_path_msg
        # #Important: Assignation of the SAME reference frame for ALL robots
        drone_path_msg.header.frame_id = tr_path_msg.header.frame_id = global_frame

        enable_motors() #Execute the functions
        t0 = rospy.Time.now().to_sec()
        while(fin):
                ex0 = ex            
                if(cambio(ex)):                    
                    deseadas = trayectoria()
                ex = np.subtract(deseadas,cord)
                funcionCosto(ex)		
                movement(PID(kp,ki,kd,ex,ex0))
                velocity_publisher.publish(vel)
                drone_path_pub.publish(drone_path_msg); tr_path_pub.publish(tr_path_msg) 
                rate.sleep()            		
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass