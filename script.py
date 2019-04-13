import io

import cv2
import numpy as np
import time
import serial
import socket




def get_irises_location(frame_gray):


    eye_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_eye.xml')
    eyes = eye_cascade.detectMultiScale(frame_gray, 1.3, 10)  # if not empty - eyes detected
    irises = []

    for (ex, ey, ew, eh) in eyes:
        iris_w = int(ex + float(ew / 2))
        iris_h = int(ey + float(eh / 2))
        irises.append([np.float32(iris_w), np.float32(iris_h)])

    return np.array(irises)



def convertToRGB(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

port = "/dev/tty.usbmodem142201"
baud = 9600

"""
ser = serial.Serial(port, baud)


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ('172.20.10.12', 10000)
sock.connect(server_address)

print("ready")
data = b''
while True:
    buf = sock.recv(1)
    if data == b'start':
        print("data received")
        print(data)
        break
    else:
        print(data)
        data += buf
"""

cam = cv2.VideoCapture(0)
print("Camera is opened:", cam.isOpened())
sleepcounter = 0
carcounterR = 0
carcounterL = 0
startTimeRight = time.time()
startTimeLeft = time.time()
elapsedRight = 0
elapsedLeft = 0
lastTimeBuzzRight = time.time()
lastTimeBuzzLeft = time.time()
starttimeeyes = time.time()
elapsedeyes = 0
lastTimeeyeBuzz = time.time()
while True:
    # Loading the image to be tested
    success, test_image = cam.read()

    if not success:
        continue
    test_image_gray = cv2.cvtColor(test_image, cv2.COLOR_BGR2GRAY)

    haar_cascade_face = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')

    haar_cascade_eyes = cv2.CascadeClassifier('data/haarcascades/haarcascade_eye.xml')

    faces_rects = haar_cascade_face.detectMultiScale(test_image_gray, scaleFactor=1.6, minNeighbors=5)

    carcascades4 = cv2.CascadeClassifier('data/haarcascades/cas4.xml')


    height, width, channels = test_image.shape

    """   
if ser.in_waiting:
        inputCom = ser.readline()
        if inputCom == b"right\n":
            roi_left = test_image_gray[0:int(height), 0:int(width/2)]
            carrects4left = carcascades4.detectMultiScale(roi_left, scaleFactor=1.2, minNeighbors=30)
            llh, llw = roi_left.shape
            cv2.rectangle(test_image, (0, 0), (llw, llh), (255, 255, 255), 2)
            print("Cars found left: ", len(carrects4left))

            if len(carrects4left) > 0 and carcounterL == 0:
                startTimeLeft = time.time()
                elapsedLeft = time.time() - startTimeLeft
                carcounterL += 1
            elif len(carrects4left) > 0 and carcounterL != 0:
                elapsedLeft = time.time() - startTimeLeft
                carcounterL += 1
            else:
                carcounterL = 0
             #   elapsedLeft = 0

            for (x, y, w, h) in carrects4left:
                cv2.rectangle(test_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if elapsedLeft > 1:
                print("Cars Left: " + str(carcounterL) + ": " + str(elapsedLeft))
                if time.time() - lastTimeBuzzLeft >= 1:
                    #ser.write(str.encode("buzz-left\n"))
                    lastTimeBuzzLeft = time.time()
            else:
                print("No Cars Left: " + str(carcounterL) + "; " + str(elapsedLeft))
        elif inputCom ==  b"left\n":
            roi_right = test_image_gray[0:int(height), int(width/2):int(width)]
            carrects4right = carcascades4.detectMultiScale(roi_right, scaleFactor=1.2, minNeighbors=30)
            rlh, rlw = roi_right.shape
            cv2.rectangle(test_image, (int(width / 2), 0), (int(width / 2) + rlw, rlh), (0, 0, 0), 2)
            print("Cars found right: ", len(carrects4right))

            if len(carrects4right) > 0 and carcounterR == 0:
                startTimeRight = time.time()
                elapsedRight = time.time() - startTimeRight
                carcounterR += 1
            elif len(carrects4right) > 0 and carcounterL != 0:
                elapsedRight = time.time() - startTimeRight
                carcounterR += 1
            else:
                carcounterR = 0
           #     elapsedRight = 0

            for (x, y, w, h) in carrects4right:
                cv2.rectangle(test_image, (x + int(width / 2), y), (x + w + int(width / 2), y + h), (255, 0, 0), 2)

            if elapsedRight > 1:
                print("Cars Right: " + str(carcounterR) + ": " + str(elapsedRight))
                if time.time() - lastTimeBuzzRight >= 1:
                 #   ser.write(str.encode("buzz-right\n"))
                    lastTimeBuzzRight = time.time()

            else:

                print("No Cars Right: " + str(carcounterR) + "; " + str(elapsedRight))

        print(inputCom)
    """
    print('Faces found: ', len(faces_rects))
    for (x, y, w, h) in faces_rects:
        cv2.rectangle(test_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        roi_gray = test_image_gray[y:y + h, x:x + w]
        roi_color = test_image[y: y + h, x:x + w]
        eyes = haar_cascade_eyes.detectMultiScale(roi_gray)

        if len(get_irises_location(roi_gray)) == 0 and len(eyes) > 0 and sleepcounter == 0:
            starttimeeyes = time.time()
            elapsedeyes = time.time() - starttimeeyes
            sleepcounter += 1
        elif len(get_irises_location(roi_gray)) == 0 and len(eyes) > 0 and sleepcounter != 0:
            elapsedeyes = time.time() - starttimeeyes
            sleepcounter += 1
        elif len(get_irises_location(roi_gray)) > 1:
            sleepcounter = 0
            elapsedeyes = 0

        print("Irises:", len(get_irises_location(roi_gray)))
        print("Eyes: ", len(eyes))

        if elapsedeyes > 2:
            print("Asleep: " + str(sleepcounter) + ": " + str(elapsedeyes))
            if time.time() - lastTimeeyeBuzz >= 1:
                #ser.write(str.encode("buzz\n"))
                lastTimeeyeBuzz = time.time()

        else:

            print("Normal: " + str(sleepcounter) + "; " + str(elapsedeyes))




        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)

    winname2 = "Driving Pal"

    cv2.namedWindow(winname2)
    cv2.moveWindow(winname2, 0, 0)
    cv2.imshow(winname2, test_image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        cam.release()
        break