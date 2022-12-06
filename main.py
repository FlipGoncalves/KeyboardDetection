# Main file
#
# Filipe Gonçalves - 98083
# Paulo Pereira - 98430

import numpy as np
import cv2
import copy

##### global variables go here
# video capture
capture = cv2.VideoCapture(0) ##### missing kinect


""" Main function """
def main():
    while True:
        ret, frame = capture.read()

        cv2.imshow('video', frame)

        k = cv2.waitKey(1)

        if k == ord("q"): 
            break


if __name__ == "__main__":
    main()