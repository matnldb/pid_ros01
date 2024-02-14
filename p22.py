#!/usr/bin/env python
import numpy as np
import rospy,tf
from math import sin, cos, pi
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

global_frame = "/world"
tr_path_msg = Path()

vel1 = Twist()
msgs_ref1 = Vector3()
drone_path_msg1 = Path()
enable1 = "uav1/enable_motors"

vel2 = Twist()
msgs_ref2 = Vector3()
drone_path_msg2 = Path()
enable2 = "uav2/enable_motors"


cord1 = [0,0,0,0]
cord2 = [0,2,0,0]
tiempo = 0
count = 0
fin = True

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

def enable_motors(SERVICE_ENABLE_MOTORS): 
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print(SERVICE_ENABLE_MOTORS+" succesfull")
        else:
            print(SERVICE_ENABLE_MOTORS+ " failed")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")

def actualiza(v,valor):
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v

def poseCallback(cord,drone_path_msg,msg,base):
    global cord1, cord2, drone_path_msg1, drone_path_msg2    
    pos = msg.pose.pose.position
    cord[:3] = [pos.x, pos.y, pos.z]       
    quater = msg.pose.pose.orientation
    quater_list = [quater.x, quater.y, quater.z, quater.w]
    (r,p,cord[3]) = euler_from_quaternion(quater_list)        
    drone_pose = PoseStamped() #Create the PoseStamped instance
    drone_pose.pose.position = pos
    drone_path_msg.poses.append(drone_pose)
    #Assignation of the TF's (used for Rviz only)
    br = tf.TransformBroadcaster()
    br.sendTransform( (cord[:3]), 
        tf.transformations.quaternion_from_euler(r,p,cord[3]), rospy.Time.now(), 
        base, global_frame )

def poseCallback1(msg): 
        global cord1, drone_path_msg1
        poseCallback(cord1,drone_path_msg1,msg,"/uav1/base_link")

def poseCallback2(msg): 
        global cord2, drone_path_msg2
        poseCallback(cord2,drone_path_msg2,msg,"/uav2/base_link")

def Saturacion(elemento, saturacion):
    for i in range(4):
        if elemento[i] > saturacion:
            elemento[i] = saturacion
        elif elemento[i] < -saturacion: 
            elemento[i] = -saturacion
    return elemento 
  
def PID(kp,ki,kd, ex,ex0,i_ex): 
    d_ex = np.subtract(ex,ex0)*100
    i_ex +=np.add(ex, ex0)*0.005
    i_ex = Saturacion(i_ex, 500)
    p = np.multiply(ex,kp)
    i = np.multiply(i_ex,ki)
    d = np.multiply(d_ex,kd)
    pi = np.add(p,i)
    pid = np.subtract(pi,d)
    return pid

def movement(v,vant_vel): 
        vant_vel.linear = actualiza(vant_vel.linear,v)
        vant_vel.angular = actualiza(vant_vel.angular,[0,0,v[3]])

def trayectoryStamped(d):
        global tr_path_msg
        tr_pose = PoseStamped()
        tr_pose.pose.position.x = d[0] 
        tr_pose.pose.position.y = d[1]
        tr_pose.pose.position.z = d[2] 
        tr_path_msg.poses.append(tr_pose)

def trayectoria():
     global tiempo,fin,tr_path_msg,i_ex1,i_ex2
     tiempo +=1     
     i_ex1 = [0,0,0,0]; i_ex2 = [0,0,0,0]     
     if tiempo == 1:
       d = [0,0,2,0]
     elif tiempo == 2:
        d =  [0,2,2,0]
     elif tiempo == 3:
       d =  [2,2,2,0]
     elif tiempo == 4:
        d =  [2,0,2,0]
     elif tiempo == 5: 
        d =  [0,0,2,0]
     elif tiempo == 6: 
        d =  [0,0,0,0]
     else: 
         fin = False
         d = [0,0,0,0]
     trayectoryStamped(d)
     return d

def trayectoria2():        
        d = [val if val == 0 else val - 0.4 for val in cord1]
        d[2:3] = cord1[2:3]
        # a = pi
        # d[0] = cord1[0]+2*cos(cord1[3]+a)
        # d[1] = cord1[1]+2*sin(cord1[3]+a)
        # d[2] = cord1[2]       
        return d    

def cambio(ex):
    global count
    if(np.all(np.abs(ex) < 0.09)):
        count+=1
    else: count = 0 
    if(count == 4):return True
    else: return False
        
def main_function():
        global msgs_ref1, rate,fin, i_ex1,i_ex2,vel1,vel2    
        i_ex1 = [0,0,0,0];i_ex2 = [0,0,0,0]
        ex = [0,0,0,0]; ex2 = [0,0,0,0]
        #kp = [0.25, 0.25, 0.5, 1.5]; kd = [0.05, 0.05, 0.05, 0]; ki = [0, 0 ,0, 0]
        kp = [ 0.29060169,  0.22351094,  0.54439942,  1.53864866]
        ki = [ 0.0493674 ,  0.05      ,  0.0378391 ,  0.00537379]
        kd = [ 0.01099189,  0.06127233,  0.0423897 ,  0.04541789] 
        deseadas = [0,0,0,0]; deseadas2 = [0,2,0,0]
        rospy.init_node("pid", anonymous=True) 
        rate = rospy.Rate(100) #Node frequency (Hz)
            
        global vel_pub1, vel_pub2
        vel_pub1 = rospy.Publisher('uav1/cmd_vel', Twist, queue_size=10) 
        vel_pub2 = rospy.Publisher('uav2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('uav1/ground_truth/state', Odometry, poseCallback1) 
        rospy.Subscriber('uav2/ground_truth/state', Odometry, poseCallback2)
        
        #To publish the paths
        drone_path_pub1 = rospy.Publisher('uav1/path', Path, queue_size=10)
        drone_path_pub2 = rospy.Publisher('uav2/path', Path, queue_size=10)
        tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10) 
        
        global drone_path_msg1, drone_path_msg2, tr_path_msg
        #Important: Assignation of the SAME reference frame for ALL robots
        drone_path_msg1.header.frame_id = drone_path_msg2.header.frame_id = tr_path_msg.header.frame_id = global_frame

        enable_motors(enable1) ; enable_motors(enable2)
        while(fin):
                ex0 = ex; ex02 = ex2           
                if(cambio(ex)):                    
                   deseadas = trayectoria()
                   deseadas2 = trayectoria2()
                ex = np.subtract(deseadas,cord1)
                ex2 = np.subtract(deseadas2,cord2)		
                movement(PID(kp,ki,kd,ex,ex0,i_ex1),vel1)
                movement(PID(kp,ki,kd,ex2,ex02,i_ex2),vel2)
                vel_pub1.publish(vel1)
                vel_pub2.publish(vel2)
                drone_path_pub1.publish(drone_path_msg1); drone_path_pub2.publish(drone_path_msg2); tr_path_pub.publish(tr_path_msg)                
                rate.sleep()            		
		        
if __name__ == '__main__':
    if os.name != 'nt': settings = termios.tcgetattr(sys.stdin)
    try: main_function()  
    except rospy.ROSInterruptException:pass