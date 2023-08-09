#! /usr/bin/python3
from typing import Iterable

import cv2
import numpy as np
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

bombInLastFrame = False

RED_MIN = np.array([115, 105, 0])
""" Minimum HSV value for "red" in image. """

RED_MAX = np.array([130, 225, 200])
""" Maximum HSV value for "red" in image. """

KERNEL = np.ones((3, 3), np.uint8)
""" Kernel for convolving image. """

BBOX_MIN_SIZE = 10
BBOX_MAX_SIZE = 55

def mask_red(frame: np.ndarray) -> np.ndarray:
    """ Filter image and return a mask where red is found. """

    # blur the image to reduce colour noise
    blurred = cv2.blur(frame, (4, 4))

    # image is actually BGR, but treating it as RGB puts the red hue in
    # the middle of the 0-180 range instead of at one end, making
    # thresholding easier
    hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, RED_MIN, RED_MAX)

    # remove noise pixels
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=5)
    return mask

def object_contours(mask: np.ndarray) -> tuple[np.ndarray]:
    """ Given a mask, return a list of object contours.

    Inner contours (holes within objects) are ignored.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def bboxes_from_contours(contours: Iterable[np.ndarray]) -> list[np.ndarray]:
    """ Given a list of contours, return all bounding boxes that are
    larger than a minimum size.

    Sort in descending order of size.
    """
    def predicate(bbox: np.ndarray) -> bool:
        _, _, w, h = bbox
        return (w >= BBOX_MIN_SIZE and w <= BBOX_MAX_SIZE) and (h >= BBOX_MIN_SIZE and h <= BBOX_MAX_SIZE)

    bbox_iter = (cv2.boundingRect(c) for c in contours)
    return [b for b in bbox_iter if predicate(b)]

def runloop(cam):
    while True:
        success, frame = cam.read()
        if not success:
            continue

        mask = mask_red(frame)
        contours = object_contours(mask)
        bboxes = bboxes_from_contours(contours)
        # overlay bounding boxes on original image
        if (len(bboxes) >= 1 and bombInLastFrame):
            GPIO.output(18, True)
        elif (len(bboxes) >= 1 and not bombInLastFrame):
            GPIO.output(18, False)
            bombInLastFrame = True
        else:
            bombInLastFrame = False
            GPIO.output(18, False)
        print(len(bboxes))
        for bbox in bboxes:
            x, y, w, h = bbox
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        #cv2.imshow("Webcam", frame)
        if cv2.waitKey(1) & 0xFF==ord('q'): # quit when 'q' is pressed
            cam.release()
            break

def main():
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)

    try:
        runloop(cam)
        
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
