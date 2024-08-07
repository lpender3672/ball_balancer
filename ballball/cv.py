
import serial
import cv2
import numpy as np
import imutils
from collections import deque
from imutils.video import VideoStream
import argparse



import time


def nothing(x):
    pass




ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())


pts = deque(maxlen=args["buffer"])

cv2.namedWindow("tracking")

cv2.createTrackbar("LH","tracking", 160, 255, nothing)
cv2.createTrackbar("UH","tracking", 180, 255, nothing)

cv2.createTrackbar("LM","tracking", 0, 255, nothing)
cv2.createTrackbar("UM","tracking", 255, 255, nothing)

cv2.createTrackbar("LE","tracking", 0, 255, nothing)
cv2.createTrackbar("UE","tracking", 255, 255, nothing)

def load_trackbars():
    try:
        with open("trackbars.txt", "r") as f:
            pass
    
    except FileNotFoundError:
        return

    with open("trackbars.txt", "r") as f:
            lines = f.readlines()
            cv2.setTrackbarPos("LH", "tracking", int(lines[0]))
            cv2.setTrackbarPos("UH", "tracking", int(lines[1]))
            cv2.setTrackbarPos("LM", "tracking", int(lines[2]))
            cv2.setTrackbarPos("UM", "tracking", int(lines[3]))
            cv2.setTrackbarPos("LE", "tracking", int(lines[4]))
            cv2.setTrackbarPos("UE", "tracking", int(lines[5]))

load_trackbars()

vs = VideoStream(src = 0, resolution=(1920, 1080), framerate=30)
vs.start()

# allow the camera or video file to warm up

centersss = []

while True:

    frame = vs.read()

    if frame is None:
        break

    LH = cv2.getTrackbarPos("LH","tracking")
    UH = cv2.getTrackbarPos("UH", "tracking")

    LM = cv2.getTrackbarPos("LM", "tracking")
    UM = cv2.getTrackbarPos("UM", "tracking")

    LE = cv2.getTrackbarPos("LE", "tracking")
    UE = cv2.getTrackbarPos("UE", "tracking")

    pinkLower = (LH, LM, LE)
    pinkUpper = (UH, UM, UE)

    #frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


    mask = cv2.inRange(hsv, pinkLower, pinkUpper)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)


    # contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # contour with the largest area
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 30:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            send = "<HI,"+ str(center[0]) + "," + str(center[1]) + ">"
            
            print(center)
            
            for i in range(1, len(pts)):

                if pts[i - 1] is None or pts[i] is None:
                    continue

                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)


    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", res)

    key = cv2.waitKey(1) & 0xFF

    # quit
    if key == ord("q"):
        break

    if key == ord("p"):
        with open("trackbars.txt", "w") as f:
            f.write(str(LH) + "\n")
            f.write(str(UH) + "\n")
            f.write(str(LM) + "\n")
            f.write(str(UM) + "\n")
            f.write(str(LE) + "\n")
            f.write(str(UE) + "\n")

        print("Trackbars saved")

    if key == ord("l"):
        load_trackbars()
        print("Trackbars loaded")
    
    if key == ord("s"):
        # screenshot 
        cv2.imwrite("screenshot.jpg", frame)
        print("Screenshot saved")


    pts.appendleft(center)
    # sleep a bit
    time.sleep(0.01)


cv2.destroyAllWindows()
