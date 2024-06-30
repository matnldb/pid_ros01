#!/usr/bin/env python
import numpy as np
import rospy
import math
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():
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

kp = [0.25, 0.25, 0.5, 1.5]
kd = [0.05, 0.05, 0.05, 0]

class Drone(object):
    def __init__(self, name):
        self.name = name
        self.vel = Twist()
        self.cord = [0, 0, 0, 0]  # [x, y, z, yaw]
        self.vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.name + '/ground_truth/state', Odometry, self.poseCallback)
        self.enable_motors()
        self.takeoff_complete = False
        self.goDown = False
        self.ex = [0, 0, 0, 0]
        self.ex0 = [0, 0, 0, 0]
        self.count = 0
        self.mse = 0
        self.band = 0

    def poseCallback(self, msg):
        pos = msg.pose.pose.position
        self.cord[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.cord[3]) = euler_from_quaternion(quater_list)

    def enable_motors(self):
        rospy.wait_for_service(self.name + '/enable_motors')
        enable_motors = rospy.ServiceProxy(self.name + '/enable_motors', EnableMotors)
        enable_motors(True)

    def take_off(self):
        if not self.takeoff_complete:
            if self.cord[2] < 2:
                self.vel.linear.z = 0.1
            else:
                self.vel.linear.z = 0.0
                self.takeoff_complete = True
        else:
            self.vel.linear.z = 0.0

    def land(self):
        if not self.goDown:
            if self.cord[2] > 0.3:
                self.vel.linear.z = -0.2
            else:
                self.vel.linear.z = 0.0
                self.goDown = True
        else:
            self.vel.linear.z = 0.0

    def calculate_error(self, desired_position):
        self.ex0 = self.ex
        self.ex = np.subtract(desired_position, self.cord)
        return self.ex

    def calcula_MSE(self, error):
        self.mse += np.sum(np.square(error))
        self.count += 1

    def reset(self):
        global cambio
        cambio = False
        self.mse = 0
        self.count = 0

    def PID(self, kp, kd, error):
        d_ex = np.subtract(error, self.ex0) * 100
        p = np.multiply(error, kp)
        d = np.multiply(d_ex, kd)
        pid = np.add(p, d)
        return pid

    def movement(self, pid):
        self.vel.linear.x = pid[0]
        self.vel.linear.y = pid[1]
        self.vel.linear.z = pid[2]
        self.vel.angular.z = pid[3]

class Lider(Drone):
    def __init__(self, name):
        super(Lider, self).__init__(name)
        self.square_trajectory = generate_square_trajectory()
        self.current_vertex = 0
        self.next_vertex = 1

    def determine_desired_pose(self, desired_angle=0):
        global cambio
        if np.all(np.abs(self.ex) < 0.05):
            if self.band != 0:
                cambio = True
            self.band += 1
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.square_trajectory)
        return self.square_trajectory[self.current_vertex]

def generate_square_trajectory():
    square_trajectory = [
        (0, 0, 2, 0),
        (0, 2, 2, 0),
        (2, 2, 2, 0),
        (2, 0, 2, 0),
    ]
    return square_trajectory

class Seguidor(Drone):
    def __init__(self, name, Lider):
        super(Seguidor, self).__init__(name)
        self.lider = Lider.name
        self.leader_position = [0, 0, 0, 0]
        rospy.Subscriber(self.lider + '/ground_truth/state', Odometry, self.leaderPoseCallback)
        self.exCol = [0, 0, 0, 0]
        self.distance_pub = rospy.Publisher(self.name + '/distance_to_leader', Float32, queue_size=10)
        self.distError_pub = rospy.Publisher(self.name + '/error_distancia', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher(self.name + '/angle_to_leader', Float32, queue_size=10)
        self.anglError_pub = rospy.Publisher(self.name + '/error_angle', Float32, queue_size=10)

        self.distance_errors = []
        self.angle_errors = []

    def leaderPoseCallback(self, msg):
        pos = msg.pose.pose.position
        self.leader_position[:3] = [pos.x, pos.y, pos.z]
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (_, _, self.leader_position[3]) = euler_from_quaternion(quater_list)

    def determine_desired_pose(self, angle, tol=1e-6, max_iter=1000):
        d = 2
        a, b, c, yaw = self.leader_position
        z_follower = c - 0.4
        cos_angle = math.cos(math.radians(angle))

        def objective(x, y):
            distance = math.sqrt((x - a)**2 + (y - b)**2 + (z_follower - c)**2)
            dot_product = a * x + b * y + c * z_follower
            norm_leader = math.sqrt(a**2 + b**2 + c**2)
            norm_follower = math.sqrt(x**2 + y**2 + z_follower**2)
            cos_actual = dot_product / (norm_leader * norm_follower)
            distance_error = abs(distance - d)
            angle_error = abs(cos_actual - cos_angle)
            return distance_error + angle_error

        x_follower = a + d * math.cos(math.radians(angle))
        y_follower = b + d * math.sin(math.radians(angle))
        step_size = 0.01

        for _ in range(max_iter):
            current_error = objective(x_follower, y_follower)
            if current_error < tol:
                break
            x_step = x_follower + step_size
            y_step = y_follower + step_size
            error_x_step = objective(x_step, y_follower)
            error_y_step = objective(x_follower, y_step)

            if error_x_step < current_error:
                x_follower = x_step
            if error_y_step < current_error:
                y_follower = y_step

        return [x_follower, y_follower, z_follower, yaw]

    def errorControler(self, desired_angle=0):
        x1, y1, z1, yaw1 = self.leader_position
        x2, y2, z2, yaw2 = self.cord

        delta_x = x2 - x1
        delta_y = y2 - y1
        delta_z = z2 - z1

        distance = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        dot_product = x1 * x2 + y1 * y2 + z1 * z2

        norm_a = np.sqrt(x1**2 + y1**2 + z1**2)
        norm_b = np.sqrt(x2**2 + y2**2 + z2**2)

        if norm_a == 0 or norm_b == 0:
            cos_theta = 1.0 if norm_a == norm_b else 0.0
        else:
            cos_theta = dot_product / (norm_a * norm_b)
            cos_theta = np.clip(cos_theta, -1, 1)

        angle_radians = np.arccos(cos_theta)
        angle_degrees = np.degrees(angle_radians)

        self.distance_pub.publish(distance)
        self.angle_pub.publish(angle_radians)

        self.distance_errors.append(distance)
        self.angle_errors.append(angle_radians)

    def calculate_statistics(self):
        distance_errors = np.array(self.distance_errors)
        angle_errors = np.array(self.angle_errors)

        distance_error_max = np.max(distance_errors)
        distance_error_min = np.min(distance_errors)
        distance_error_mean = np.mean(distance_errors)
        distance_error_mse = np.mean(np.square(distance_errors))

        angle_error_max = np.max(angle_errors)
        angle_error_min = np.min(angle_errors)
        angle_error_mean = np.mean(angle_errors)
        angle_error_mse = np.mean(np.square(angle_errors))

        return {
            "distance_error_max": distance_error_max,
            "distance_error_min": distance_error_min,
            "distance_error_mean": distance_error_mean,
            "distance_error_mse": distance_error_mse,
            "angle_error_max": angle_error_max,
            "angle_error_min": angle_error_min,
            "angle_error_mean": angle_error_mean,
            "angle_error_mse": angle_error_mse
        }

def main_function():
    global cambio, kp, kd
    lado = 0
    MSE2 = 0
    rospy.init_node("pid", anonymous=True)
    rate = rospy.Rate(50)

    lider = Lider("uav1")
    seguidor1 = Seguidor("uav2", lider)
    while not rospy.is_shutdown():
        lider.take_off()
        seguidor1.take_off()

        seguidor1.errorControler(45)

        if lider.takeoff_complete and seguidor1.takeoff_complete:
            deseadas_lider = lider.determine_desired_pose(0)
            deseadas_seguidor = seguidor1.determine_desired_pose(45)

            error_lider = lider.calculate_error(deseadas_lider)
            error_seguidor1 = seguidor1.calculate_error(deseadas_seguidor)

            seguidor1.calcula_MSE(error_seguidor1)

            pl = lider.PID(kp, kd, error_lider)
            ps1 = seguidor1.PID(kp, kd, error_seguidor1)

            lider.movement(pl)
            seguidor1.movement(ps1)

            if lado < 4:
                if cambio:
                    MSE2 += seguidor1.mse / seguidor1.count
                    lado += 1
                    seguidor1.reset()
            else:
                lider.land()
                seguidor1.land()
                if lider.goDown and seguidor1.goDown:
                    print("MSE: ", MSE2 / 4)
                    stats = seguidor1.calculate_statistics()
                    print("Statistics: ", stats)
                    return MSE2 / 4

        lider.vel_pub.publish(lider.vel)
        seguidor1.vel_pub.publish(seguidor1.vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
