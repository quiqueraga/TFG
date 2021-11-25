import cv2
import numpy as np
from pyzbar.pyzbar import decode

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



