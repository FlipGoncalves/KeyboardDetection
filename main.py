#### Main file
#
## Filipe Gonçalves - 98083
## Paulo Pereira - 98430


# imports
import numpy as np
import cv2
import copy
import json
from functools import partial
import pcd

# range for keyboard segmentation
try:
    with open("limits.json") as f:
        data = json.load(f)
except:
    print(f"Error opening file, exiting...")
    exit(1)

# video capture
capture = cv2.VideoCapture('VideoColor.avi')


""" Applies binary threshould operation according to given ranges """
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


""" Keyboard centroid detection """
def findCentroid(img_processed):
    connectivity = 4

    # Perform the operation
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(cv2.cvtColor(img_processed, cv2.COLOR_BGR2GRAY), connectivity, cv2.CV_32S)

    height, width = img_processed.shape[0:2]

    middle_centroid = (int(width/2), int(height/2))
    cv2.circle(img_processed, middle_centroid, 1, (0,255,0), -1)

    # get labels by max area value
    max_labels = sorted([(i, stats[i, cv2.CC_STAT_AREA], [(stats[i][0],stats[i][1]),(stats[i][0]+stats[i][2],stats[i][1]+stats[i][3])]) for i in range(0, nb_components)], key=lambda x: x[1], reverse=True)

    # print green boxes around the objects found
    objects = []
    for label in max_labels:
        # whole space -> we dont care
        if label[0] <= 1 or label[1] < 10:
            continue

        # draw the bounding rectangele around each object
        # cv2.rectangle(img_processed, label[2][0], label[2][1], (0,255,0), 2)
        objects.append(label)

    if len(objects) > 1:
        # objects is a list of all the usefull objects found in the processed image
        # biggest is the special keys: ESC and F1-11
        cv2.rectangle(img_processed, objects[0][2][0], objects[0][2][1], (255,0,0), 2)

        # # second biggest is the space bar
        # cv2.rectangle(img_processed, objects[1][2][0], objects[1][2][1], (0,0,255), 2)

    return objects[0]


""" Main function """
def main():

    lock = False

    cv2.namedWindow("Image")

    # get information for the keyboard and save it
    ret, frame = capture.read()

    # frames shape
    height,width,depth = frame.shape

    # mask to get only the middle rectangle with the keyboard
    mask = np.zeros(frame.shape[:2],np.uint8)
    mask[int(height/3):int(height/3)+int(height/3),int(width/3):int(width/3)+int(width/3)] = 255
    frame = cv2.bitwise_and(frame,frame,mask = mask)

    # binary threshold image
    img_processed = processImage(data["limits"], frame)

    # find and trim centroids
    keyboard_rect = findCentroid(img_processed)

    # # set keyboard values to squares
    # keyboard_values = keyboardValues(keyboard)

    cv2.imshow("Keyboard processed", img_processed)

    print("Press any key to start")
    cv2.waitKey(-1)

    cv2.destroyWindow("Keyboard processed")

    # pcd variables
    base_num_points = 0
    key_pressed = False
    key_pressed_started = 0
    frame_number = 1

    print("Starting....\n\n")

    while True:

        # get frame from the video
        ret, frame = capture.read()

        # wait for possible voluntary code break
        k = cv2.waitKey(1)

        if k == ord("q") or frame is None: 
            break

        # identify if hand touches the base board
        base_num_points, keyboard_pressed = pcd.get_keypress(base_num_points, frame_number)

        # if the keyboard is not in the process of being pressed, but it should
        if keyboard_pressed and not key_pressed:
            key_pressed_started = frame_number
            key_pressed = True
        
        # finishing key press - in average it takes 20 frames, as it is less than a second (30fps)
        if key_pressed and frame_number > key_pressed_started + 20:
            key_pressed = False

        # pressing key - in average it takes a little less than half the time of a key press, as there is always a little time of preassure
        if key_pressed and frame_number == key_pressed_started + 8:
            # identify if the pressure is in the same spot as the keyboard

            # mask to get only the middle rectangle with the keyboard
            mask = np.zeros(frame.shape[:2],np.uint8)
            mask[int(height/3):int(height/3)+int(height/3),int(width/3):int(width/3)+int(width/3)] = 255
            frame = cv2.bitwise_and(frame,frame,mask = mask)        

            # binary threshold image
            img_processed = processImage(data["limits"], frame)
            # find and trim centroids
            centroid = findCentroid(img_processed)

            # if the frame centroid is different than the keyboard centroid, then the hand is in front of the keyboard
            # hand in front of the keyboard + hand in the base board of the keyboard => key press !!
            if centroid[2][0] != keyboard_rect[2][0] or centroid[2][1] != keyboard_rect[2][1]:
                print("Key pressed !!!")

        if k != -1:
            lock = not lock
        
        if not lock:
            # show the final image after processing, noise removal and find the centroids
            cv2.imshow('Image', frame)

        # frame number
        frame_number += 1


if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()