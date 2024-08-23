
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

qr = cv2.QRCodeDetector()

cmtx, dist = np.load("calibration.npy", allow_pickle=True)

def undistort(frame):
    h,  w = frame.shape[:2]
    alpha = 0.5
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cmtx, dist, (w,h), alpha, (w,h))
    dst = cv2.undistort(frame, cmtx, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    return dst

# allow the camera or video file to warm up

ser = serial.Serial('COM3', 115200)

plate_center = (300, 297)
camera_angle_rel_plate = []
time_last = None

centersss = []

while True:

    frame = vs.read()

    if frame is None:
        break

    ret_qr, points_qr = qr.detect(frame)

    if ret_qr:
        qr_edges = np.array([   [0,0,0],
                        [0,1,0],
                        [1,1,0],
                        [1,0,0] ], dtype = 'float32').reshape((4,1,3))

        ret, rvec, tvec = cv2.solvePnP(qr_edges, points_qr, cmtx, dist)

        unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))

        if ret:
            axis_points, jac = cv2.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
            
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

            if len(axis_points) > 0:
                axis_points = axis_points.reshape((4,2))

                origin = (int(axis_points[0,0]),int(axis_points[0,1]) )

                # get angle of rotated bar code on the plate
                angle = np.arctan2(axis_points[1,1] - axis_points[0,1], axis_points[1,0] - axis_points[0,0])
                camera_angle_rel_plate.append(angle)

                for p, c in zip(axis_points[1:], colors):
                    p = (int(p[0]), int(p[1]))

                    #Sometimes qr detector will make a mistake and projected point will overflow integer value. We skip these cases. 
                    if origin[0] > 5*frame.shape[1] or origin[1] > 5*frame.shape[1]: break
                    if p[0] > 5*frame.shape[1] or p[1] > 5*frame.shape[1]: break

                    cv2.line(frame, origin, p, c, 2)

    if len(camera_angle_rel_plate) < 10:
        continue

    frame = undistort(frame)

    #ret, rvec, tvec = cv2.solvePnP(qr_edges, points_qr, cmtx, dist)

    LH = cv2.getTrackbarPos("LH","tracking")
    UH = cv2.getTrackbarPos("UH", "tracking")

    LM = cv2.getTrackbarPos("LM", "tracking")
    UM = cv2.getTrackbarPos("UM", "tracking")

    LE = cv2.getTrackbarPos("LE", "tracking")
    UE = cv2.getTrackbarPos("UE", "tracking")

    pinkLower = (LH, LM, LE)
    pinkUpper = (UH, UM, UE)

    #frame = imutils.resize(frame, width=600)
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


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

        if radius > 20:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            center = (int(x), int(y))

            pos_rel_plate = (x - plate_center[0], y - plate_center[1])
            smoothed_angle = np.mean(camera_angle_rel_plate[-10:])
            pos_rel_plate_prime = (pos_rel_plate[0]*np.cos(smoothed_angle) - pos_rel_plate[1]*np.sin(smoothed_angle),
                                   pos_rel_plate[0]*np.sin(smoothed_angle) + pos_rel_plate[1]*np.cos(smoothed_angle))
            
            pos_mm = np.array(pos_rel_plate_prime) * 90 / 150
            
            if time_last is None:
                time_last = time.time()
                pos_last = pos_mm
                continue

            pos_mm = (pos_mm + pos_last) / 2 # smoothing

            time_now = time.time()
            dt = time_now - time_last
            time_last = time_now

            dposdt_mmpersec = (pos_mm - pos_last) / dt
            pos_last = pos_mm

            print(pos_mm)

            # update control loop
            x = np.array([pos_mm[0], dposdt_mmpersec[0], pos_mm[1], dposdt_mmpersec[1]]) / 1000 # convert to meters
            K = np.array([  [-2.3,   -0.5,    0.0000,   -0.0000],
                            [-0.0000,   -0.0000,   -2.3,   -0.5]])
            
            u = np.dot(K, x)
            u_deg = np.rad2deg(u)
            ser.write(f"A{u_deg[0]} B{u_deg[1]}\n".encode())
            # send to arduino

            #serial.write(f"A{u[0]} B{u[1]}\n".encode())

            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
            #cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # display smoothed_angle
            smoothed_angle_degrees = 180 + np.rad2deg(smoothed_angle)
            cv2.putText(frame, f"angle: {smoothed_angle_degrees:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            # display speed and position
            speed = np.linalg.norm(dposdt_mmpersec)
            cv2.putText(frame, f"speed: {speed:.2f} mm/s", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        
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

cv2.destroyAllWindows()
