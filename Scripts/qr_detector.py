import cv2
import math
import numpy as np
from pyzbar import pyzbar

def qr_detector(img):

    detector = cv2.QRCodeDetector()

    bool, points = detector.detect(img)

    cv2.drawContours(img,[np.int32(points)],0,(0,0,255),2)

    return img, points

def main():

    img = cv2.imread('movil1.jpg')
    img2 = cv2.imread('movil21.jpg')

    img, points = qr_detector(img)
    img2, points2 = qr_detector(img2)

    cv2.imshow('QR',img)
    cv2.imshow('QR2',img2)

    cv2.waitKey(0)
    cv2.destroyAllWindows()