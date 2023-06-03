#!/usr/bin/env python
import numpy as np
import rospy,tf
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
    #drone 1
    SERVICE_ENABLE_MOTORS = "uav1/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav1: Enable motors successful\n")
        else:
            print("uav1: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")
    
    #drone 2
    SERVICE_ENABLE_MOTORS = "uav2/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav2: Enable motors successful\n")
        else:
            print("uav2: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")#######################################################################################

msgs_ref1 = Vector3()
vel1 = Twist()
drone_path_msg1 = Path(); drone_path_msg2 = Path()
tr_path_msg = Path()
global_frame = "/world"
msgs_ref2 = Vector3()
vel2 = Twist()

drone_path_msg = Path()
tr_path_msg = Path()
global_frame = "/world"
# posiciones actuales(x,y,z, quaternion_yaw) y deseadas 
cord = [0,0,0,0]; i_ex = [0,0,0,0]
cord2 = [0,2,0,0]; i_ex2 = [0,0,0,0]
tiempo = 0; tiempo2 = 0
count = 0
fin = True

def actualiza(v,valor):
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v

def poseCallback1(msg): #Callback function to get the drone posture
        global cord
        pos = msg.pose.pose.position
        cord[:3] = [pos.x, pos.y, pos.z]       
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r,p,cord[3]) = euler_from_quaternion(quater_list) #Euler angles are given in radians       
        drone_pose1 = PoseStamped() #Create the PoseStamped instance
        drone_pose1.pose.position = pos
        drone_path_msg1.poses.append(drone_pose1)
        #Assignation of the TF's (used for Rviz only)
        br = tf.TransformBroadcaster()
        br.sendTransform( (cord[:3]), 
            tf.transformations.quaternion_from_euler(r,p,cord[3]), rospy.Time.now(), 
            "/uav1/base_link", global_frame )
       

def poseCallback2(msg): #Callback function to get the drone posture
        global cord2
        pos = msg.pose.pose.position
        cord2[:3] = [pos.x, pos.y, pos.z]       
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r,p,cord2[3]) = euler_from_quaternion(quater_list) #Euler angles are given in radians               
        drone_pose2 = PoseStamped() #Create the PoseStamped instance
        drone_pose2.pose.position = pos
        drone_path_msg2.poses.append(drone_pose2)
        #Assignation of the TF's (used for Rviz only)
        br = tf.TransformBroadcaster()
        br.sendTransform( (cord2[:3]), 
            tf.transformations.quaternion_from_euler(r,p,cord2[3]), rospy.Time.now(), 
            "/uav2/base_link", global_frame )

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

def PID2(kp,ki,kd, ex,ex0):
    global i_ex2
    d_ex = np.subtract(ex,ex0)*100
    i_ex2 = i_ex2+np.add(ex, ex0)*0.005
    i_ex2 = Saturacion(i_ex2, 500)
    p = np.multiply(ex,kp)
    i = np.multiply(i_ex,ki)
    d = np.multiply(d_ex,kd)
    pi = np.add(p,i)
    pid = np.subtract(pi,d)
    return pid

def movement(v): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel1.linear = actualiza(vel1.linear,v)
        vel1.angular = actualiza(vel1.angular,[0,0,v[3]])  

def movement2(v): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel2.linear = actualiza(vel2.linear,v)
        vel2.angular = actualiza(vel2.angular,[0,0,v[3]]) 

def trayectoryStamped(d):
        global tr_path_msg
        tr_pose = PoseStamped()
        tr_pose.pose.position.x = d[0] 
        tr_pose.pose.position.y = d[1]
        tr_pose.pose.position.z = d[2] 
        tr_path_msg.poses.append(tr_pose)

def trayectoria():
     global tiempo,fin,tr_path_msg
     tiempo +=1
     
     print(tiempo,fin)
     if tiempo == 1:
       d = [0,0,2,0]
     elif tiempo == 2:
        d =  [0,2,2,0]
     elif tiempo == 3:
       d =  [2,2,2,0]
     elif tiempo == 3:
        d =  [2,0,2,0]
     elif tiempo == 4: 
        d =  [0,0,2,0]
     elif tiempo == 5: 
        d =  [0,0,0,0]
     else: 
         fin = False
         d = [0,0,0,0]
     trayectoryStamped(d)
     return d

def trayectoria2():
    global tiempo2,fin,tr_path_msg
    b = [val if val == 0 else val - 0.4 for val in cord]
    return b    

def cambio(ex):
    global count
    if(np.all(np.abs(ex) < 0.09)):
        count+=1
    else: count = 0 
    if(count == 4):return True
    else: return False
        
def main_function():
        global msgs_ref1, rate,fin      
        
        ex = [0,0,0,0]; ex2 = [0,0,0,0]
        kp = [0.25, 0.25, 0.5, 1.5]; kd = [0.05, 0.05, 0.05, 0]; ki = [0, 0 ,0, 0]
        deseadas = [0,0,0,0]; deseadas2 = [0,2,0,0]
        rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
        rate = rospy.Rate(50) #Node frequency (Hz)
            
        global vel_pub1, vel_pub2
        vel_pub1 = rospy.Publisher('uav1/cmd_vel', Twist, queue_size=10) #To publish in the topic
        vel_pub2 = rospy.Publisher('uav2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('uav1/ground_truth/state', Odometry, poseCallback1) #To subscribe to the topic
        rospy.Subscriber('uav2/ground_truth/state', Odometry, poseCallback2)
        
        #To publish the paths
        drone_path_pub1 = rospy.Publisher('uav1/path', Path, queue_size=10)
        drone_path_pub2 = rospy.Publisher('uav2/path', Path, queue_size=10)
        tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10) 
        
        global drone_path_msg1, drone_path_msg2, tr_path_msg
        #Important: Assignation of the SAME reference frame for ALL robots
        drone_path_msg1.header.frame_id = drone_path_msg2.header.frame_id = tr_path_msg.header.frame_id = global_frame

        enable_motors() #Execute the functions
        while(fin):
                ex0 = ex; ex02 = ex2           
                if(cambio(ex)):                    
                   deseadas = trayectoria()
                   deseadas2 = trayectoria2()
                ex = np.subtract(deseadas,cord)
                ex2 = np.subtract(deseadas2,cord2)		
                movement(PID(kp,ki,kd,ex,ex0))
                movement2(PID2(kp,ki,kd,ex2,ex02))
                vel_pub1.publish(vel1)
                vel_pub2.publish(vel2)
                drone_path_pub1.publish(drone_path_msg1); drone_path_pub2.publish(drone_path_msg2); tr_path_pub.publish(tr_path_msg)                
                rate.sleep()            		
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass