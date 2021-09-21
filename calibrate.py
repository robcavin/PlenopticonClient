import cv2 as cv
import numpy as np
import glob

images = glob.glob("*png")
print(images)

objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

obj_locations = []
img_locations = []

for i in range(4):
    obj_locations.append([])
    img_locations.append([])

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for image in images :
    full_img = cv.imread(image)
    for offset in range(4):
        img = full_img[:,offset*1280:offset*1280+1280]
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (8,6), None)

        if ret == True :
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            img_locations[offset].append(corners2)
            obj_locations[offset].append(objp)

            cv.drawChessboardCorners(img, (8,6), corners2, ret)
            cv.imshow("matches", img)
            cv.waitKey()

        else :
            print("Could not find corners")

for offset in range(4) :
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(obj_locations[offset], img_locations[offset], gray.shape[::-1], None, None)
    print(mtx)

cv.waitKey()