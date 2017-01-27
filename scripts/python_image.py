#!/usr/bin/env python
import roslib
roslib.load_manifest('polarcam_gige')
import sys
import io
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function

class python_image:
    def __init__(self):

        self.image_pub = rospy.Publisher("python_image", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            
        except CvBridgeError as e:
            print(e)

       #(rows, cols, channels) = cv_image.shape
       # if cols > 60 and rows > 60:
       #     cv2.circle(cv_image, (50, 50), 10, 255)
        cv2.imshow("Image window", cv_image)
        Path = '/home/lemaitre/Documents/test/images/'
        cv2.imwrite(Path.__add__('IMG' + str(id(cv_image)) + '.tiff'), cv_image)
        cv2.waitKey(3)
        print('image is displaying')
        print (id(cv_image))
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = python_image()
    rospy.init_node('python_image', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

