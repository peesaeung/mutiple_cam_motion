# USAGE
# python multi_cam_motion.py

# import the necessary packages
from __future__ import print_function
import datetime
import time
import os
import numpy as np
import cv2
import imutils
from pyimagesearch.basicmotiondetector2 import BasicMotionDetector
# from pyimagesearch.tempimage import TempImage
from imutils.video import VideoStream

def locs(locsx, frame):
    if len(locsx) > 0:
            # initialize the minimum and maximum (x, y)-coordinates,
            # respectively
        (minX, minY) = (np.inf, np.inf)
        (maxX, maxY) = (-np.inf, -np.inf)

            # loop over the locations of motion and accumulate the
            # minimum and maximum locations of the bounding boxes
        for l in locsx:
            (x, y, w, h) = cv2.boundingRect(l)
            (minX, maxX) = (min(minX, x), max(maxX, x + w))
            (minY, maxY) = (min(minY, y), max(maxY, y + h))

            # draw the bounding box
        cv2.rectangle(frame, (minX, minY), (maxX, maxY), (0, 0, 255), 3)
        return

# initialize the video streams and allow them to warmup
print("[INFO] starting cameras...")
webcam = VideoStream(src=0, resolution=(800, 600)).start()
picam = VideoStream(usePiCamera=True, resolution=(800, 600)).start()
time.sleep(2.0)

# initialize the two motion detectors
camMotion = BasicMotionDetector()
piMotion = BasicMotionDetector()
lastUploaded = datetime.datetime.now()
motionCounter = 0

# loop over frames from the video streams
while True:
    # initialize the list of frames that have been processed
        # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 500 pixels
    frame_1 = webcam.read()
    frame_1 = imutils.resize(frame_1, width=640)
    frame_2 = picam.read()
    frame_2 = imutils.resize(frame_2, width=640)

    # convert the frame to grayscale, blur it slightly, update
    # the motion detector
    gray_1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.GaussianBlur(gray_1, (21, 21), 0)
    gray_2 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2GRAY)
    gray_2 = cv2.GaussianBlur(gray_2, (21, 21), 0)

    # we should allow the motion detector to "run" for a bit
    # and accumulate a set of frames to form a nice average
#    if total < 32:
 #       continue

    # otherwise, check to see if motion was detected
    locs_1, D1 = camMotion.update(gray_1)
    locs_2, D2 = piMotion.update(gray_2)
    locs(locs_1, frame_1)
    locs(locs_2, frame_2)

#    if len(locs_1) > 0:
            # initialize the minimum and maximum (x, y)-coordinates,
            # respectively
 #           (minX, minY) = (np.inf, np.inf)
  #          (maxX, maxY) = (-np.inf, -np.inf)

            # loop over the locations of motion and accumulate the
            # minimum and maximum locations of the bounding boxes
   #         for l in locs_1:
    #            (x, y, w, h) = cv2.boundingRect(l)
     #           (minX, maxX) = (min(minX, x), max(maxX, x + w))
      #          (minY, maxY) = (min(minY, y), max(maxY, y + h))

            # draw the bounding box
       #     cv2.rectangle(frame_1, (minX, minY), (maxX, maxY),
        #        (0, 0, 255), 3)
#    if len(locs_2) > 0:
            # initialize the minimum and maximum (x, y)-coordinates,
            # respectively
 #           (minX, minY) = (np.inf, np.inf)
  #          (maxX, maxY) = (-np.inf, -np.inf)

            # loop over the locations of motion and accumulate the
            # minimum and maximum locations of the bounding boxes
   #         for l in locs_2:
    #            (x, y, w, h) = cv2.boundingRect(l)
     #           (minX, maxX) = (min(minX, x), max(maxX, x + w))
      #          (minY, maxY) = (min(minY, y), max(maxY, y + h))

            # draw the bounding box
       #     cv2.rectangle(frame_2, (minX, minY), (maxX, maxY),
        #        (0, 0, 255), 3)
    # draw the timestamp on the frame
#    total += 1
    timestamp = datetime.datetime.now()
    ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
    cv2.putText(frame_1, ts, (10, frame_1.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    cv2.putText(frame_2, ts, (10, frame_2.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    # notify when detected
    # check to see if enough time has passed between uploads

    if D1 is True and D2 is True:
        if (timestamp - lastUploaded).seconds >= 1:
            # increment the motion counter
            motionCounter += 1

            # check to see if the number of frames with consistent motion is
            # high enough
            if motionCounter >= 5:
                print(ts)
                path = "./Pictures/{timestamp}/".format(timestamp=ts)
                os.makedirs(path)
                path1 = "Pictures/{timestamp}/path1.jpg".format(timestamp=ts)
                path2 = "Pictures/{timestamp}/path2.jpg".format(timestamp=ts)
                cv2.imwrite(path1, frame_1)
                cv2.imwrite(path2, frame_2)
                # update the last uploaded timestamp and reset the motion
                # counter
                lastUploaded = timestamp
                motionCounter = 0

    # otherwise, the room is not occupied
    else:
        motionCounter = 0
#        cv2.imwrite('path1.jpg', frame_1)
#        cv2.imwrite('path2.jpg', frame_2)

    # show the frame
    cv2.imshow("Webcam", frame_1)
    cv2.imshow("Picam", frame_2)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
print("[INFO] cleaning up...")
cv2.destroyAllWindows()
webcam.stop()
picam.stop()
