# Keyboard Callback
#
# Filipe Gonçalves - 98083
# Paulo Pereira - 98430

import numpy as np
import cv2
import copy
import json
from functools import partial

##### global variables go here
# video capture
capture = cv2.VideoCapture(0) ##### missing kinect

try:
    with open("limits_callback.json") as f:
        data = json.load(f)
except:
    print(f"Error opening file, exiting...")
    exit(1)


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


""" Main function """
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
        if label[0] <= 1 or label[1] < 500:
            continue

        # draw the bounding rectangele around each object
        cv2.rectangle(img_processed, label[2][0], label[2][1], (0,255,0), 2)
        objects.append(label)

    if len(objects) > 1:
        # objects is a list of all the usefull objects found in the processed image
        # biggest is the special keys: ESC and F1-11
        cv2.rectangle(img_processed, objects[0][2][0], objects[0][2][1], (255,0,0), 2)

        # second biggest is the space bar
        cv2.rectangle(img_processed, objects[1][2][0], objects[1][2][1], (0,0,255), 2)

    return objects


keyboard = json.load(open("keyboard.json"))
shift_pressed = False
control_pressed = False
caps_pressed = False

""" Transforms the pixels pressed with the left button of the mouse into real keyboard keys """
def mouse_handler(event, x, y, flags, params, keyboard):
    global shift_pressed, control_pressed, caps_pressed

    if event == cv2.EVENT_LBUTTONDOWN:
        # # for the handle keyboard function
        # print(x, y)
        # letter = input(f"What should i save ({x}, {y}) as? ")
        # keyboard[letter] = (x,y)

        for key, label in keyboard.items():
            if x > label[2][0][0] and y > label[2][0][1]:
                if x < label[2][1][0] and y < label[2][1][1]:
                    if key == "space":
                        print(" ")
                    elif "shift" in key:
                        # dunno what to do here
                        shift_pressed = not shift_pressed
                        print("Shift button pressed")
                    elif "control" in key:
                        # dunno what to do here
                        control_pressed = not control_pressed
                        print("Control button pressed")
                    elif "alt" in key:
                        # dunno what to do here
                        print("Alt button pressed")
                    elif key == "caps":
                        caps_pressed = not caps_pressed
                        print("Caps Lock button pressed")
                    elif key == "escape":
                        # dunno what to do here
                        print("Escape button pressed")
                        exit(1)
                    elif key == "Power":
                        # dunno what to do here
                        print("Power button pressed")
                    elif key == "backspace":
                        # dunno what to do here
                        print("Backspace button pressed")
                    elif key == "enter":
                        # dunno what to do here
                        print("Enter button pressed")
                    elif key == "tab":
                        # dunno what to do here
                        print("Tab button pressed")
                    else:
                        print(key.upper() if caps_pressed else key)

""" Turns the keyboard values, calibrated before, into actual labels """
def keyboardValues(centroids):
    real_keyboard = {}
    for key, value in keyboard.items():
        (x,y) = eval(value)
        for label in centroids:
            if x > label[2][0][0] and y > label[2][0][1]:
                if x < label[2][1][0] and y < label[2][1][1]:
                    real_keyboard[key] = label

    return real_keyboard


""" Main function """
def main():

    centroids = []
    lock = False

    frame = cv2.imread("keyboard.jpg")

    cv2.namedWindow("Processed Image")

    while True:
        # ret, frame = capture.read()

        # cv2.imshow('video', frame)

        k = cv2.waitKey(1)

        if k == ord("q"): 
            break

        # binary threshold image
        img_processed = processImage(data["limits"], frame)

        # find and trim centroids
        centroids = findCentroid(img_processed)

        # set keyboard values to squares
        keyboard = keyboardValues(centroids)

        # set callback with real keyboard values
        cv2.setMouseCallback("Processed Image", partial(mouse_handler, keyboard=keyboard))

        if k != -1:
            lock = not lock
        
        if not lock:
            # show the final image after processing, noise removal and find the centroids
            cv2.imshow('Processed Image', img_processed)

    # cv2.waitKey(-1)

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()