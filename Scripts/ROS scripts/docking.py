#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import time
from homography import homography_method
from pyzbar.pyzbar import decode

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class camera():

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback) # Suscripción al topic de la camara
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1) # Publicamos en el topic de la velocidad de movimiento del robot
    self.cont = 0
    self.inicio = time.time()


  def callback(self,data):
    twist = Twist()
    bridge = CvBridge()
    
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    image = cv_image

    dest = cv2.imread('/home/quique/catkin_ws/src/qr/destHomography2.png') # Cargamos la imagen en la que tenemos al robot ya acoplado. Adecuar segun acoplamiento del robot
    qr_result = decode(image) # Detección del QR

    if not qr_result: # QR no encontrado en el docking
      twist.linear.x = 0
      twist.angular.z = 0.5
      self.cmd_vel_pub.publish(twist)
    else: # QR encontrado en el docking

      # Obtenemos y dibujamos el contorno del QR
      qr_data = qr_result[0].data

      (x, y, w, h) = qr_result[0].rect
      points = np.array([np.array([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])])

      cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 4) # cv2.drawContours(image, points, 0, (0,0,255), 4) # Metodo alternativo

      # Computamos la homografia
      pr_qr = decode(dest)
      (x1, y1, w1, h1) = pr_qr[0].rect
      points_dest = np.array([np.array([(x1, y1), (x1 + w1, y1), (x1 + w1, y1 + h1), (x1, y1 + h1)])])      
      cv2.drawContours(dest, points_dest, 0, (0,0,255), 4)
      H, status = cv2.findHomography(points_dest,points)
      
      K = np.array([[530.4669406576809, 0.0, 320.5], [0.0, 530.4669406576809, 240.5], [0.0, 0.0, 1.0]]) # Matriz parametros intrinsecos de la camara
      #print(K)

      b, rot, tras = homography_method(H,K)
      rot_filtered = np.around(rot,1)
      tras_filtered = np.around(tras,1)

      # Obtenemos los momentos para computar el centroide
      M = cv2.moments(points)
      if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
        # BEGIN CONTROL
        if np.all(np.abs(tras_filtered) <= 0.1) and np.abs(rot_filtered[1]) <= 0.1: # Condición de parada
          twist.linear.x = 0          
          self.cmd_vel_pub.publish(twist)
          if self.cont > 0:
            print("Acoplamiento finalizado")
          self.cont += 1
        else:
          """ if rot_filtered[1] > 0.1:
            twist.angular.z = -0.1
          elif rot_filtered[1] < 0.1:       # Metodo alternativo
            twist.angular.z = 0.1
          else:
            twist.linear.x = 0.2 """
          if rot_filtered[1] > 0.1:
            twist.angular.z = -0.1
            twist.linear.x = 0.1
          elif rot_filtered[1] < 0.1:
            twist.angular.z = 0.1
            twist.linear.x = 0.1
          elif np.all(np.abs(tras_filtered) < 0.1):
            if rot_filtered[1] > 0:
              twist.angular.z = -0.1
            elif rot_filtered[1] < 0:
              twist.angular.z = 0.1
          else:
            twist.linear.x = 0.2
          self.cmd_vel_pub.publish(twist)
        # END CONTROL
      text = "{}".format(qr_data)
      cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imshow("Camera output", image) # Vista de la camara del robot en tiempo real
    cv2.imshow("Homography output", dest) # Vista de la camara del robot estando ya acoplado

    cv2.waitKey(5)

def main():
	camera()
	
	try:
	  rospy.spin()
	except rospy.is_shutdown:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('Docking', anonymous=False)
    main()
