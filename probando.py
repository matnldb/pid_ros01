#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

refs = [0,0,0,0] # alamacena las coordenadas del punto de partida
actRef = [0,0,0,0] # coordenadas transformadas a partir del punto de partida
first_callback = True # para establecer el punto de partida una sola vez
i_ex = [0,0,0,0]
vel = Twist()

def actualiza(v,valor): #esta funcion recibe como parametro un vector y devuelve un elemento de odometria
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v
 
def callback(msg):  # funcion que es actualiza la pose actual 
    global refs, first_callback, actRef
    
    pos = msg.pose.pose.position
    quater = msg.pose.pose.orientation
    quater_list = [quater.x, quater.y, quater.z, quater.w]

    if first_callback: # la primera vez que se llama la pose actual se convierte en el origen de referncia
        refs[:3] = [pos.x, pos.y, pos.z]
        (r,p,refs[3]) = euler_from_quaternion(quater_list) 
        first_callback = False

    actRef[:3] = [pos.x, pos.y, pos.z] # Obtiene las coordenadas actuales  en todo momento  
    (r,p,actRef[3]) = euler_from_quaternion(quater_list)    
   

def Saturacion(elemento, saturacion): 
    for i in range(4):
        if elemento[i] > saturacion:
            elemento[i] = saturacion
        elif elemento[i] < -saturacion: 
            elemento[i] = -saturacion
    return elemento

def PID(kp,ki,kd, ex,ex0):
    global i_ex
    
    d_ex = np.subtract(ex,ex0)*100 #calcula la derivada del error dx=0.01
    i_ex = i_ex+np.add(ex, ex0)*0.005 #iex por el metodo del trapecio dx = 0.01
    i_ex = Saturacion(i_ex, 500) #limita el error por inetgracion
    p = np.multiply(ex,kp)
    i = np.multiply(i_ex,ki)
    d = np.multiply(d_ex,kd)
    pi = np.add(p,i)
    pid = np.add(pi,d)
    if ex[0]>0:
        print("error",ex[0],p[0]) 
        print("der",d_ex[0],d[0])
        # print("int",i_ex[0],p[0])  
        print("vel",pid[0])
    else:
     pid = [0,0,0,0] 
    return pid

def mov(v): #control del movimiento a partir de las Vx, Vy, Vz, Wz
        global vel
        vel.linear = actualiza(vel.linear,v)
        vel.angular = actualiza(vel.angular,[0,0,v[3]])        
    
def trayectoria(): #se modificara la funcion para establecer una trayectoria  
     global tiempo
     tiempo +=1
     if tiempo == 1:
         pass
     else: pass     

def cambio(ex): # condicion para considerar el error lo suficientemente small
    if(np.all(np.abs(ex) < 0.09)):
        return True
    else: return False

def main_function():
    global actRef, vel, refs
    ex = [0,0,0,0]; ex0 = [0,0,0,0] 
    #definicion de las constantes para el control en x-y-z-yaw
    kp = [2,2,1,0.5]; kd=[0.085,0.085,1,0.5]
    ki = [0,0,0,0]; des = [0,0,0,0]
    rospy.init_node('pid', anonymous=True);rate = rospy.Rate(10)
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ardrone/odometry', Odometry, callback)
    while not rospy.is_shutdown(): 
        if not first_callback:
            des = np.copy(refs) 
            des[0]+=1
        ex = np.subtract(des,actRef) # calcula el error
        ppid = PID(kp,ki,kd,ex,ex0)
        ex0 = np.copy(ex)
        print("  ")
        mov(Saturacion(ppid,0.9)) 
            
        cmd_vel_publisher.publish(vel)
        rate.sleep() 

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
         pass