#/usr/bin/env python

import cv2
import numpy as np
import jetson.utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def show_image(img):
    """
        Displays images

        Parameters
        +--------------------+
        img: The image to be displayed. Should be of type 'ndarray'
    """
    cv2.imshow('image', img)
    cv2.waitKey(1)

def get_result(img):
    """
        Given an image from 'situation_awareness' ROS node report back results such as
        bounding box coordinates of a given light source w/in Gazebo

        Parameters
        +--------------------+
        img: 'Image' message type that was forwarded from 'situational_awareness' node for
                processing

        Return value(s)
        +--------------------+
        data: Python dictionary that stores model results in key value pairs. Keys:
                {"class_id", "confidence", "predicted_class", "left", "top", "right",
                "bottom", "width", "height", "area", "center"}
    """
    # Initialize return value
    data = None
    # Initialize CvBridge class
    bridge = CvBridge()
    # TODO: Import exported custom object detection model
    # model = import(custom_model)
    # Do necessary preprocessing on 'img' parameter to feed into model
    try:
        # Convert ROS Image class to OpenCV2 image
        cv2_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        # Convert OpenCV2 image (numpy ndarray) to cudaImage
        cuda_mem = jetson.utils.cudaFromNumpy(cv2_img)
    except CvBridgeError as e:
        print(e)
    # Copy above image
    drawImg = cv2_img
    # Display the image
    show_image(drawImg)
    # TODO: Produce results by passing processed image into the model
    # detections = model(cuda_mem, img.width, img.height)
    # TODO: Parse results from predictions/detections
    # if (len(detections) > 0):
        # data = {}
        # detection = detections[0]
        # data['class_id'] = detection.ClassID
        # data['confidence'] = detection.Confidence
        # data['predicted_class'] = model.GetClassDesc(class_id)
        # data['left'] = detection.Left
        # data['top'] = detection.Top
        # data['right'] = detection.Right
        # data['bottom'] = detection.Bottom
        # data['width'] = detection.Width
        # data['height'] = detection.Height
        # data['area'] = detection.Area
        # data['center_x'] = detection.Center[0]
        # data['center_y'] = detection.Center[1]
    # TODO: Return results in a data structure of some sort for later 
    # processing by sender
    return data
