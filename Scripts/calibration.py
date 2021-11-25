# %%
import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import matplotlib

def patterDeffining(corners_r, corners_c):
    # Prepare object points matrix
    objp = np.zeros((corners_r*corners_c, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[:corners_r, :corners_c].T.reshape(-1, 2)
    return objp


def printCorners(corners_r, corners_c,objp):
    images = glob.glob('calibration/640/image*.jpg')
    # Define lists for storing both the object points and the corresponding image points
    objpoints = []
    imgpoints = []

    # Find corners in each pattern
    for file in images:
        img = cv2.imread(file)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
        gray, (corners_r, corners_c), None)

        # If corners are found, match object points with image corners
        if ret == True:

            objpoints.append(objp)
            imgpoints.append(corners)

            # show the first image with corners detected
            cv2.drawChessboardCorners(img, (corners_r, corners_c), corners, ret)
            plt.figure()
            plt.title(file)
            plt.imshow(img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        else:
            """ print("[NOK]") """

    """ print("Number of patterns found:", len(objpoints)) """
    return objpoints, imgpoints, gray, corners

def calibration(objpoints, imgpoints, gray):
    # Apply Zang's method for calibrate the camera
    _, intrinsic, distortion, rotation, translation = cv2.calibrateCamera(
        objpoints, imgpoints, (gray.shape[1], gray.shape[0]), None, None)

    tot_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
        objpoints[i], rotation[i], translation[i], intrinsic, distortion)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error

    """ print("Mean error: ", tot_error/len(objpoints)) """
    return intrinsic, distortion, rotation, translation

def draw(img, corners, imgpts):

    corner = tuple(corners[0].ravel())
    img = cv2.line(img, np.int32(corner), np.int32(tuple(imgpts[0].ravel())), (255,0,0), 5)
    img = cv2.line(img, np.int32(corner), np.int32(tuple(imgpts[1].ravel())), (0,255,0), 5)
    img = cv2.line(img, np.int32(corner), np.int32(tuple(imgpts[2].ravel())), (0,0,255), 5)

    return img
    
corners_r = 8
corners_c = 6
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = patterDeffining(corners_r,corners_c)
objpoints, imgpoints, gray, corners = printCorners(corners_r, corners_c,objp)
cameraMatrix, distCoeffs, _, _ = calibration(objpoints, imgpoints, gray)
# %%
