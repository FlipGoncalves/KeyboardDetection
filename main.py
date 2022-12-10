# Main file
#
# Filipe Gonçalves - 98083
# Paulo Pereira - 98430

import numpy as np
import cv2
import copy
import json

##### global variables go here
# video capture
capture = cv2.VideoCapture(0) ##### missing kinect
desvio_centroids = 10

try:
    with open("limits.json") as f:
        data = json.load(f)
except:
    print(f"Error opening file, exiting...")
    exit(1)

def processImage(ranges, image):
    """
    # Applies binary threshould operation according to given ranges
    """

    # processing
    mins = np.array([ranges['B']['min'], ranges['G']['min'], ranges['R']['min']])
    maxs = np.array([ranges['B']['max'], ranges['G']['max'], ranges['R']['max']])

    # mask
    mask = cv2.inRange(image, mins, maxs)
    # conversion from numpy from uint8 to bool
    mask = mask.astype(bool)

    # process the image
    image_processed = copy.deepcopy(image)
    image_processed[np.logical_not(mask)] = 0

    # get binary image with threshold the values not in the mask
    _, image_processed = cv2.threshold(image_processed, 1, 255, cv2.THRESH_BINARY)

    return image_processed


def findCentroid(img_processed):
    connectivity = 4  
    centroids = None

    # Perform the operation
    _, _, _, centroids = cv2.connectedComponentsWithStats(cv2.cvtColor(img_processed, cv2.COLOR_BGR2GRAY), connectivity, cv2.CV_32S)

    for center in centroids:
        cv2.circle(img_processed, (int(center[0]), int(center[1])), 1, (255,0,0), -1)

    # print(len(centroids))
    # if (len(centroids) - 100) > desvio_centroids:
    #     print("hmm")
    
    # return centroids


""" Main function """
def main():

    while True:
        ret, frame = capture.read()

        cv2.imshow('video', frame)

        k = cv2.waitKey(1)

        if k == ord("q"): 
            break

        # binary threshold image
        img_processed = processImage(data["limits"], frame)

        # find centroids
        findCentroid(img_processed)

        # show the final image after processing, noise removal and find the centroids
        cv2.imshow('Processed Image', img_processed)


if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()