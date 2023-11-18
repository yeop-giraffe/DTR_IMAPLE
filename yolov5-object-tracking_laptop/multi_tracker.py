''' This whole Script is for the open cv tracking
It is mean to operate with the yolo detections after every 30 frames '''
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
    help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
    help="OpenCV object tracker type")
args = vars(ap.parse_args())

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.legacy.TrackerCSRT_create,
    "kcf": cv2.legacy.TrackerKCF_create,
    "boosting": cv2.legacy.TrackerBoosting_create,
    "mil": cv2.legacy.TrackerMIL_create,
    "tld": cv2.legacy.TrackerTLD_create,
    "medianflow": cv2.legacy.TrackerMedianFlow_create,
    "mosse": cv2.legacy.TrackerMOSSE_create
}

# Initialize OpenCV's multi-object tracker
multi_tracker = cv2.legacy.MultiTracker_create()

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)
# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

yolo_detections = [
    [0.775*255, 0.894792*255, 0.128125*255, 0.210417*255],  # Box 1
    [0.157031*255, 0.759375*255, 0.267188*255, 0.460417*255],  # Box 2
]

# Run yolov5 and get detections to feed into tracker

# loop over frames from the video stream
while True:
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame

    if frame is None:
        break

    frame = imutils.resize(frame, width=600)

    # Create and add trackers for each initial bounding box
    if len(multi_tracker.getObjects()) == 0:
        for box in yolo_detections:
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            tracker.init(frame, box)
            multi_tracker.add(tracker, frame, box)

    # grab the updated bounding box coordinates (if any) for each
    # object that is being tracked
    success, boxes = multi_tracker.update(frame)

    # loop over the bounding boxes and draw them on the frame
    for box in boxes:
        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

if not args.get("video", False):
    vs.stop()
else:
    vs.release()

cv2.destroyAllWindows()

''' Tracking Code ends here'''
