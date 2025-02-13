#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class ObjectDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image  = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # crop the image
        cropped_img = cv_image[80:300, 100:520]

        # convert image to grayscale
        gray_image = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)

        # The parameters 5 and 15 allow to remove noises on the image. User can play with them to get a better image
        # create a mask based on threshold of grayscale
        mask_image = cv.adaptiveThreshold(gray_image, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 5, 15)
        
        """
            If some segments of the image are not linked due to the low quality of the image, we can use the 
            'cv.CHAIN_APPROX_SIMPLE' parameter to approximate all the segments like a chain in a way to get 
            one single contour
        """
         # find contours
        contours, _ = cv.findContours(mask_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        """
            The printed array represent all the contours. So all the vectors in x and y contained in the array, 
            represent all the points that compose the contours of the displayed image in the mask. Those points 
            are the edge of our object that we want to detect.
        """
        print("contours: ", contours)

        """ 
            Draw a polyline that reflects these contours around the object. 
            [255, 0, 0] indicates the color (blue in this case) of the polyline
        """
        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [255, 0, 0], 1)

        """
            a cnt value corresponds to an array of point vectors in x and y. This array defines a closed surface 
            which we can calculate its area.
            contours =
            [array([[[248, 100]], [[247, 101]], [[247, 102]], [[245, 104]], [[244, 104]], [[236, 112]], [[236, 113]],
            [[235, 114]], [[235, 115]], [[232, 118]], [[241, 127]], [[242, 127]], [[243, 126]], [[244, 126]], [[245, 125]],
            [[246, 125]], [[247, 124]], [[247, 123]], [[242, 119]], [[246, 123]], [[244, 125]], [[243, 125]], [[242, 126]],
            [[241, 125]], [[240, 125]], [[234, 119]], [[234, 117]], [[237, 114]], [[238, 115]], [[237, 114]], [[237, 113]],
            [[236, 112]], [[241, 107]], [[242, 107]], [[245, 104]], [[247, 102]], [[248, 102]], [[249, 101]], [[259, 111]],
            [[258, 112]], [[258, 113]], [[249, 122]], [[250, 122]], [[251, 121]], [[251, 120]], [[253, 118]], [[254, 118]],
            [[254, 117]], [[255, 116]], [[256, 116]], [[256, 115]], [[258, 113]], [[259, 113]], [[260, 112]], [[250, 102]],
            [[250, 101]],[[249, 100]]], dtype=int32)]
            So a cnt defines one contour
        """
        """
            The below peace of code iterate on all the contours, compute the area of each contour, check if there is a 
            contour with an area > 20, we approximize the polylines in such a way that we are gonna group this contour 
            into one object
        """
        """
            This algorimth is doing some kind of filtration in case we have a noisy image in other to get just object
            that respect some criteria (area > 20)
        """
        object_detected = []
        for cnt in contours:
            area = cv.contourArea(cnt)  #compute the area of the contour
            if area > 20:
                # 0.03 is a parameter that defines how we are going to approximize (more or less)
                cnt = cv.approxPolyDP(cnt, 0.03*cv.arcLength(cnt, True), True)  #group the contour into one object 
                object_detected.append(cnt)
        print("how many object I detect: ", len(object_detected))
        print(object_detected)

        """
            Now we draw a rectangular in the object that we have detected and we detect the center in x and y 
            coordinates with the respect of the camera link and the orientation of this box directly on the image 
            that we have detected thanks to the object detection
        """
        for cnt in object_detected:  #for filtered objects that respect the above criteria
            #look for a rectangular that fits inside the contour cnt and save that rectangular inside the variable rect
            rect = cv.minAreaRect(cnt)
            """
                We get the properties of the rectangular: (x_center, y_center) indicates the center of the rectangular
                (w, h) indicates its width and height and then we have its orientation
            """
            (x_center, y_center), (w,h), orientation = rect
            box = cv.boxPoints(rect)  #draw a box related to the detected rectangular
            box = np.int0(box)  #we use numpy to approximate all the points of the above box into integers
            cv.polylines(cropped_img, [box], True, (0, 255,0),1)
            #put the text
            cv.putText(cropped_img, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
            cv.circle(cropped_img, (int(x_center), int(y_center)), 1, (0,255,0), thickness=-1) #draw a circle



        cv.imshow('original', cv_image)
        cv.imshow('cropped', cropped_img)
        cv.imshow('gray',gray_image)
        cv.imshow("mask",mask_image)
        cv.waitKey(1)


def main():
    object_detection = ObjectDetection()
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()