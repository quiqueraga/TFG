#!/usr/bin/env python

import rospy
import numpy as np
import time

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg): # Metodo para obtener la pose actual del robot y su angulo
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("Navigation")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point() # Establecemos el punto aproximado al que debe navegar el robot
goal.x = 1.0
goal.y = 4.0
inc_x = goal.x - x
inc_y = goal.y - y

while np.abs(inc_x) > 0.01 and np.abs(inc_y) > 0.01: # Bucle para navegar hasta estar suficientemente cerca del punto propuesto
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1: # Umbral para rotar
        if (angle_to_goal - theta) > 0:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.0
            speed.angular.z = -0.3
    else:                               # Si se pasa el umbral, avanzar en esa direccion
        speed.linear.x = 0.3
        speed.angular.z = 0.0

    pub.publish(speed)

speed.linear.x = 0
speed.angular.z = 0
pub.publish(speed)
print("Navegacion finalizada")
