# Calibrating the camera
#
# Filipe Gonçalves - 98083
# Paulo Pereira - 98430

import numpy as np
import cv2

##### global variables go here
# chessboard size
board_h = 9
board_w = 6
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((board_w*board_h,3), np.float32)
objp[:,:2] = np.mgrid[0:board_w,0:board_h].T.reshape(-1,2)
# video capture
capture = cv2.VideoCapture(0) ##### missing kinect
# calibration variables
calibrated = False
count = 1
# the number of frames we would like to get to calibrate the camera
num_calibration = 10

""" Calibration with Chessboard Detection """
def FindAndDisplayChessboard(img):
    # Find the chess board corners
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # try to find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (board_w,board_h),None)

    # found the chessboard
    if ret == True:
        print("Found Chessboard")

    # return if found and the corners
    return ret, corners


""" Main function """
def main():

    # loop for all the frames received from the camera while the camera is not calibrated
    while True:

        # get the frame
        ret, frame = capture.read()

        # show the image
        cv2.imshow('video', frame)

        # get the key if pressed
        k = cv2.waitKey(1)

        # if key is "q" then user wants to quit
        if k == ord("q"): 
            break

        # if we got some event key, it means the user wants to select the image to calibrate the camera
        if k != -1:
            # find the corners of the chessboard in this frame
            ret, corners = FindAndDisplayChessboard(frame)

            # if corners were found
            if ret == True:
                print(f"\t Image {count} calibrated")

                # get the 3d and 2d points
                objpoints.append(objp)
                imgpoints.append(corners)

                # update count variable
                count += 1

                # if we got enought frames for calibration
                if count > num_calibration:
                    break
            else:
                print("\tCorners not found, Please try again!")

    # calibrate the camera
    print("Calibrating camera...")
    retval, cameraMatrix, distortion, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame.shape[:2], 0, 0)
    print("Calibration ended!!")
    # save params to a file
    print("Saving file to camera_params.npz")
    np.savez('camera_calibration.npz', 
        intrinsics=[cameraMatrix[0][0], cameraMatrix[1][1], cameraMatrix[0][2], cameraMatrix[1][2]], 
        distortion=distortion)


if __name__ == "__main__":
    main()