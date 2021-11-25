import cv2
import math
import numpy as np
from qr_detector import qr_detector
import calibration as cab

def homography_method(H, K, approx_T=None): # Metodo para filtrar las 4 posibles soluciones que da decomposeHomographyMat()
    num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)

    count = 0
    valid = []

    for a in range(num):
        if Ts[a][2][0] < 0 or np.dot(np.array([0,0,1]), Ns[a]) > 0:
            # print(a)
            # print(cv2.Rodrigues(Rs[a])[0])
            # print(Ts[a])
            valid.append(a)
            count += 1

    # print(count)

    if len(valid) < 1:
        return False, None, None

    R_vector = cv2.Rodrigues(Rs[-1])[0]
    T_vector = Ts[-1]
    return True, R_vector, T_vector

img = cv2.imread('movil1.jpg')
img2 = cv2.imread('movil21.jpg')

img, points = qr_detector(img)
img2, points2 = qr_detector(img2)

H, status = cv2.findHomography(points,points2,method=cv2.RANSAC)
print("Homography whit findH: ") 
print(H)

# Prueba de que la homografia se realiza correctamente
""" res = cv2.getPerspectiveTransform(points,points2) 
print(res)                 
print("----------------------")                       
trans = cv2.warpPerspective(img,res,(480,640))
img, points = qr_detector(img)
img2, points2 = qr_detector(img2)

cv2.imshow('QR',img)
cv2.imshow('QR2',img2)
cv2.imshow("Transformation",trans)

cv2.waitKey(0)
cv2.destroyAllWindows() """

# Usamos la calibracion para obtener la matriz K
corners_r = 6
corners_c = 8
objp = cab.patterDeffining(corners_r,corners_c)
objpoints, imgpoints, gray, corners = cab.printCorners(corners_r, corners_c,objp)
K, distorsion, rvecs, tvecs	= cab.calibration(objpoints, imgpoints, gray)

b, rotation, traslation = homography_method(H,K)
