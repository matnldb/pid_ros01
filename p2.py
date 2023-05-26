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
m_ex, m_dex, m_iex, m_pid, m_ex0, vel = Twist(), Twist(), Twist(), Twist(), Twist(), Twist()

# Global variables
cord = [0, 0, 0, 0]  # Current positions (x, y, z, quaternion_yaw)
deseadas = [0, 0, 0, 0]  # Desired positions (x, y, z, quaternion_yaw)
ex = [0, 0, 0, 0]  # Current error (ex)
ex0 = [0, 0, 0, 0]  # Previous error (ex0)
d_ex = [0, 0, 0, 0]  # Derivative of error (d_ex)
i_ex = [0, 0, 0, 0]  # Integral of error (i_ex)
i_saturacion = 500  # Saturation for integral term
vel = Twist()  # Control velocities
msgs_ref = Vector3()  # Reference positions

# PSO parameters
min_bound = [0.1, 0.1, 0.1, 0.1]  # Minimum bounds for kp, ki, kd
max_bound = [2.0, 2.0, 2.0, 2.0]  # Maximum bounds for kp, ki, kd

def actualiza(v, valor):
    v.x = valor[0]
    v.y = valor[1]
    v.z = valor[2]
    return v

def poseCallback(msg):
    global cord
    pos = msg.pose.pose.position
    cord[:3] = [pos.x, pos.y, pos.z]
    quater = msg.pose.pose.orientation
    quater_list = [quater.x, quater.y, quater.z, quater.w]
    (r, p, cord[3]) = euler_from_quaternion(quater_list)

def refCallback(msg):
    global deseadas
    deseadas[:3] = [msg.x, msg.y, msg.z]

def Saturacion(elemento, saturacion):
    for i in range(4):
        if elemento[i] > saturacion:
            elemento[i] = saturacion
        elif elemento[i] < -saturacion:
            elemento[i] = -saturacion
    return elemento

def error():
    global ex, ex0, d_ex, i_ex
    ex = np.subtract(deseadas, cord)  # Calculate current error
    d_ex = np.subtract(ex, ex0) * 100  # Calculate derivative of error
    i_ex = i_ex + np.add(ex, ex0) * 0.005  # Calculate integral of error
    ex0 = ex  # Update previous error
    i_ex = Saturacion(i_ex, i_saturacion)  # Limit the integral error

def PID(x):
    kp = x[0]
    ki = x[1]
    kd = x[2]
    p = np.multiply(ex, kp)
    i = np.multiply(i_ex, ki)
    d = np.multiply(d_ex, kd)
    pi = np.add(p, i)
    pid = np.subtract(pi, d)
    return pid

def objective_function(x):
    global ex, ex0, d_ex, i_ex
    error()
    return np.mean(np.square(PID(x)))

def optimize_pid():
    options = {'c1': 0.5, 'c2': 0.3, 'w': 0.9}
    bounds = (min_bound, max_bound)
    xopt = ps.pso(objective_function, bounds[0], bounds[1], options=options)
    return xopt

def movement(v):
    vel.linear = actualiza(vel.linear, v)
    vel.angular = actualiza(vel.angular, [0, 0, v[3]])

def trayectoria():
    global msgs_ref
    if msgs_ref.x == 0 and msgs_ref.y == 0 and msgs_ref.z == 0:
        msgs_ref = actualiza(msgs_ref, [0, 0, 2])
    elif msgs_ref.x == 0 and msgs_ref.y == 0 and msgs_ref.z == 2:
        msgs_ref = actualiza(msgs_ref, [0, 2, 2])
    elif msgs_ref.x == 0 and msgs_ref.y == 2 and msgs_ref.z == 2:
        msgs_ref = actualiza(msgs_ref, [2, 2, 2])
    elif msgs_ref.x == 2 and msgs_ref.y == 2 and msgs_ref.z == 2:
        msgs_ref = actualiza(msgs_ref, [2, 0, 2])
    elif msgs_ref.x == 2 and msgs_ref.y == 0 and msgs_ref.z == 2:
        msgs_ref = actualiza(msgs_ref, [0, 0, 2])
    else:
        msgs_ref = actualiza(msgs_ref, [0, 0, 0])    

def cambio(ex):
    global count
    if np.all(np.abs(ex) < 0.05):
        count += 1
    else:
        count = 0
    if count == 10:
        trayectoria()

def main_function():
    global msgs_ref, deseadas, ex, ex0, rate
    msgs_ref = actualiza(msgs_ref, [0, 0, 0])
    counter = 0
    key = ''

    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    ref_pub = rospy.Publisher("/referencias", Vector3, queue_size=100)
    error_pub = rospy.Publisher("/errores", Twist, queue_size=100)
    derivadas = rospy.Publisher("/derivadas", Twist, queue_size=100)
    ex0_pub = rospy.Publisher("/ex0", Twist, queue_size=100)
    integrales = rospy.Publisher("/integrales", Twist, queue_size=100)
    pid_pub = rospy.Publisher("/pid_pub", Twist, queue_size=100)

    rospy.Subscriber('/ground_truth/state', Odometry, poseCallback)
    rospy.Subscriber("/referencias", Vector3, refCallback, queue_size=100)

    enable_motors()

    while key != 'q':
        error()
        xopt = optimize_pid()
        print(xopt)
        PID_output = PID(xopt)
        movement(PID_output)
        velocity_publisher.publish(vel)
        ref_pub.publish(msgs_ref)
        ex0_pub.publish(actualiza(m_ex0, ex0))
        error_pub.publish(actualiza(m_ex, ex))
        derivadas.publish(actualiza(m_dex, d_ex))
        integrales.publish(actualiza(m_iex, i_ex))
        pid_pub.publish(actualiza(m_pid, PID_output))
        if counter == 50:
            counter = 0
        else:
            counter += 1
        rate.sleep()
        key = getKey()
        cambio(ex)

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
