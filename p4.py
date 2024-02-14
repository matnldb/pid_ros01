#!/usr/bin/env python

# Este codigo ejecuta un metodo de PSO para de forma recurrirsiva intentar 
# determinar los mejores valores para el controlador PID a partir de lograr
# el minimo valor en la funcion de costo.

import numpy as np  # se importa la libreria para operaciones  artimeticas con vectores
import rospy # biblioteca de ROS-python para las tareas de comunicacion
import pyswarm as ps # proporciona una implementacion de enjambre de particulas 
                     # para encontrar soluciones mediante la busqueda y ajuste de candidatas a lo largo de multiples iteraciones
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from nav_msgs.msg import Odometry # para la captura de los mensajes de posicion y velocidad del VANT
from geometry_msgs.msg import Twist, Vector3 # Twist>informacion sobre la velocidad lineal y angular 3D. Vector3 desribe un vector
from tf.transformations import euler_from_quaternion #tf se utiliza para gestionar transformaciones y relaciones entre marcos de referencia.

#######################################################################################

# funcion para manejar eventos del teclado
import sys, select, os #Handling command line arguments
if os.name == 'nt': # verifica si el sistema operativo es Win2
  import msvcrt
else:
  import tty, termios   # se utilizan para configurar y manipular las caracteristicas de la terminal,

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
########################################################

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
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
cord = [0,0,0,0]; i_ex = [0,0,0,0] 
ciclo = True
tiempo = 0

def actualiza(v,valor):  # recibe un vector3 para actualizar una variable de 3 dimensiones de ROS
      v.x = valor[0]
      v.y = valor[1]
      v.z = valor[2]
      return v

def poseCallback(msg): # Recibo por parametro un mensaje tipo Twist que contiene la pose
        global cord
        pos = msg.pose.pose.position    # de la pose leo las coordenadas
        cord[:3] = [pos.x, pos.y, pos.z] # asigna valores a los 3 primeros elementos de cord      
        quater = msg.pose.pose.orientation  # de la pose leo la orientacion *(en quaterniones) 
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (r,p,cord[3]) = euler_from_quaternion(quater_list) # asigno a las variables r,p,yaw(en este caso la cuarta coordenada) los valores de la transformacion de quaterniones a Euler Angles      


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
     global tiempo,ciclo,i_ex
     i_ex = [0,0,0,0]
     tiempo +=1
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
         ciclo = False
         tiempo = 0
         return [0,0,0,0]     

def cambio(ex): # funcion para cambio de convergencia, cuando el error tiende a 0.09
    if(np.all(np.abs(ex) < 0.09)):
        return True
    else: return False

def funcionObjetivo(x):
    # recibe como parametro un arreglo que contiene en cada iteracion valores
    # aleatorios de kp,ki,kd correspondientes a [x,y,z,yaw]
     
    global i_ex, velocity_publisher,ciclo, tiempo
    # Desestructuracion de la entrada como ganancias PID
    kp = x[:4]
    ki = x[4:8]
    kd = x[8:12]
    ciclo = True  # para indicar el fin de la iteraccion
    i_ex = [0,0,0,0] # se le asigna un 
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
    while(ciclo):
            ex0 = ex
            index += 1
            if(cambio(ex)):
                deseadas = trayectoria()
            ex = np.subtract(deseadas,cord)		
            movement(PID(kp,ki,kd,ex,ex0))
            velocity_publisher.publish(vel)
            vector_traspuesto = ex.reshape(-1, 1)
            escalar = np.dot(ex,vector_traspuesto)[0]
            J+= (escalar+J0)*0.005
            J0 = escalar           
            rate.sleep()
    print(J)
    return J     	
        
def main_function():
        global rate	
        rospy.init_node("pid", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
        rate = rospy.Rate(50) #Node frequency (Hz
        rospy.Subscriber('/ground_truth/state',Odometry, poseCallback)
             # crea un suscriptor que escucha el topico "/ground_truth/state" para mensajes del tipo "Odometry" y especifica que 
             # #cuando se recibe un mensaje en ese topico, se llamar a la funcion "poseCallback" para manejar el mensaje.
        
        # arreglo de posibles soluciones para el iterador PSO
        inferior = [0.2, 0.2, 0.4, 1.4,0.0, 0.0, 0.0, 0,0, 0 ,0, 0]
        superior = [0.3, 0.3, 0.6, 1.6,0.05, 0.05, 0.05, 0.1,0.1, 0.1 ,0.1, 0.1]

        enable_motors() # Encender el motor
        xopt, fopt = ps.pso(funcionObjetivo, inferior, superior) # devuelve los valores optimos y el valor al cual se converge
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