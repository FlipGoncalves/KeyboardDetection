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

try:
    with open("limits.json") as f:
        data = json.load(f)
except:
    print(f"Error opening file, exiting...")
    exit(1)


""" # Applies binary threshould operation according to given ranges """
def processImage(ranges, image):
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


""" Main function """
def findCentroid(img_processed):
    connectivity = 4
    centroids = None

    # Perform the operation
    nb_components, _, stats, centroids = cv2.connectedComponentsWithStats(cv2.cvtColor(img_processed, cv2.COLOR_BGR2GRAY), connectivity, cv2.CV_32S)

    height, width = img_processed.shape[0:2]

    middle_centroid = (int(width/2), int(height/2))
    cv2.circle(img_processed, middle_centroid, 1, (0,255,0), -1)

    cent_temp = sorted(centroids, key=lambda x: ((x[0]-middle_centroid[0])**2 + (x[1]-middle_centroid[1])**2)**(1/2))
    # print(cent_temp)

    for center in cent_temp[:50]:
        cv2.circle(img_processed, (int(center[0]), int(center[1])), 1, (0,0,255), -1)
    
    # print(centroids)

    return cent_temp


""" Main function """
def main():

    count = 0
    centroids = []
    k1 = 0
    lock = False

    while True:
        ret, frame = capture.read()

        cv2.imshow('video', frame)

        k = cv2.waitKey(1)

        if k == ord("q"): 
            break

        # binary threshold image
        img_processed = processImage(data["limits"], frame)

        # if k == ord("p") or k1 == ord("p"):
        #     k1 = k

        #     if count >= 10:
        #         # average centroids
        #         break

        #     # find  and trim centroids
        #     centroids.append(findCentroid(img_processed))

        #     count += 1

        # find  and trim centroids
        centroids = findCentroid(img_processed)
        
        if k != -1:
            lock = not lock
        
        if not lock:
            # show the final image after processing, noise removal and find the centroids
            cv2.imshow('Processed Image', img_processed)

    # print(centroids)
    # for i in centroids:
    #     print(len(i))


if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()