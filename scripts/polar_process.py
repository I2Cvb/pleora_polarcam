#!/usr/bin/python

# -*- coding: utf-8 -*-
"""
Creating a python node which process the images captaured from the camera
simulatneously and show them in the separate window

@author: olivierm, mrastgoo
"""
# ROS libraries and moudules
import roslib; roslib.load_manifest('pleora_polarcam')
import rospy 
from sensor_msgs.msg import Image
 
# Python libraries and modules 
from os.path import join, exists, split
from os import makedirs
import argparse
import numpy as np
from scipy.signal import convolve2d
from skimage.transform import resize 
import cv2 
from cv_bridge import CvBridge, CvBridgeError 

# Definition of polar_image class 
class polar_image():
    """Class that describes pixelated images, Stokes parameters and
    Polarized parameters. """

    def __init__(self, image, method='superpixel'):
        """Method for initialization"""
        self.raw = image
        self.row = np.shape(image)[0]
        self.col = np.shape(image)[1]
        self.quad = np.vstack((np.hstack((image[0::2, 0::2],
                                          image[0::2, 1::2])),
                               np.hstack((image[1::2, 1::2],
                                          image[1::2, 0::2]))))
        self.images = []
        self._set_method(method)

    def _get_method(self):
        """ getter method for the method parameter"""
        return self._method

    def _set_method(self, method):
        """ Interpolation methods based on Ratliff 2009, Gruev 2011 (bilinear
        bicubic), and Gruev 2013 (gradient-based)"""
        self._method = method
        print self._method
        if method[:7] == 'ratliff' and 1 <= int(method[7]) <= 4:

            if method[7] == '1':
                kernels = [np.array([[1, 0], [0, 0.]]),
                           np.array([[0, 1], [0, 0.]]),
                           np.array([[0, 0], [0, 1.]]),
                           np.array([[0, 0], [1, 0.]])]
            elif method[7] == '2':
                kernels = [np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0.]]),
                           np.array([[0, 0, 0], [1, 0, 1], [0, 0, 0]])/2.,
                           np.array([[1, 0, 1], [0, 0, 0], [1, 0, 1]])/4.,
                           np.array([[0, 1, 0], [0, 0, 0], [0, 1, 0]])/2.]
            elif method[7] == '3':
                b = np.sqrt(2)/2/(np.sqrt(2)+np.sqrt(10)/2)
                a = np.sqrt(10)/2/(np.sqrt(2)+np.sqrt(10)/2)

                kernels = [np.array([[0, b, 0, 0],
                                     [0, 0, 0, 0],
                                     [0, a, 0, b],
                                     [0, 0, 0, 0]]),
                           np.array([[0, 0, b, 0],
                                     [0, 0, 0, 0],
                                     [b, 0, a, 0],
                                     [0, 0, 0, 0]]),
                           np.array([[0, 0, 0, 0],
                                     [b, 0, a, 0],
                                     [0, 0, 0, 0],
                                     [0, 0, b, 0]]),
                           np.array([[0, 0, 0, 0],
                                     [0, a, 0, b],
                                     [0, 0, 0, 0],
                                     [0, b, 0, 0]])]
            elif method[7] == '4':
                c = np.sqrt(2)/2/(3*np.sqrt(2)/2+np.sqrt(2)/2+np.sqrt(10))
                b = np.sqrt(10)/2/(3*np.sqrt(2)/2+np.sqrt(2)/2+np.sqrt(10))
                a = 3*np.sqrt(2)/2/(3*np.sqrt(2)/2+np.sqrt(2)/2+np.sqrt(10))

                kernels = [np.array([[0, b, 0, c],
                                     [0, 0, 0, 0],
                                     [0, a, 0, b],
                                     [0, 0, 0, 0]]),
                           np.array([[c, 0, b, 0],
                                     [0, 0, 0, 0],
                                     [b, 0, a, 0],
                                     [0, 0, 0, 0]]),
                           np.array([[0, 0, 0, 0],
                                     [b, 0, a, 0],
                                     [0, 0, 0, 0],
                                     [c, 0, b, 0]]),
                           np.array([[0, 0, 0, 0],
                                     [0, a, 0, b],
                                     [0, 0, 0, 0],
                                     [0, b, 0, c]])]
            Is = []
            for k in kernels:
                Is.append(convolve2d(self.raw, k, mode='same'))

            offsets = [[(0, 0), (0, 1), (1, 1), (1, 0)],
                       [(0, 1), (0, 0), (1, 0), (1, 1)],
                       [(1, 1), (1, 0), (0, 0), (0, 1)],
                       [(1, 0), (1, 1), (0, 1), (0, 0)]]

            self.images = []
            for (j, o) in enumerate(offsets):
                self.images.append(np.zeros(self.raw.shape))
                for ide in range(4):
                    self.images[j][o[ide][0]::2, o[ide][1]::2] = Is[ide][o[ide]
                                                                         [0]::2,
                                                                         o[ide]
                                                                         [1]::2]
        elif method == 'bilinear':
            self.images = [resize(self.raw[0::2, 0::2].astype(float),
                                   (self.row, self.col)),
                           resize(self.raw[0::2, 1::2].astype(float),
                                  (self.row, self.col)),
                           resize(self.raw[1::2, 1::2].astype(float),
                                  (self.row, self.col)),
                           resize(self.raw[1::2, 0::2].astype(float),
                                  (self.row, self.col))]
        elif method == 'bicubic':
            self.images = [resize(self.raw[0::2, 0::2].astype(float),
                                   (self.row, self.col), order=3),
                           resize(self.raw[0::2, 1::2].astype(float),
                                  (self.row, self.col), order=3),
                           resize(self.raw[1::2, 1::2].astype(float),
                                  (self.row, self.col), order=3),
                           resize(self.raw[1::2, 0::2].astype(float),
                                  (self.row, self.col), order=3)]            

        elif method == 'superpixel':
            self.images = [self.raw[0::2, 0::2].astype(float),
                           self.raw[0::2, 1::2].astype(float),
                           self.raw[1::2, 1::2].astype(float),
                           self.raw[1::2, 0::2].astype(float)]

        else:
            self.images = []

    method = property(_get_method, _set_method)

 
    @property
    def polarization(self):
        """ Property that computes the polar params from the 4 images """
        Js = self.images
        inten = (Js[0]+Js[1]+Js[2]+Js[3])/2.
        aop = (0.5*np.arctan2(Js[1]-Js[3], Js[0]-Js[2]))
        dop = np.sqrt((Js[1]-Js[3])**2+(Js[0]-Js[2])**2)/(Js[0]+Js[1]+Js[2]+
                                                          Js[3]+
                                                          np.finfo(float).eps)*2
        return (inten, aop, dop)

    @property
    def rgb(self):
        """ Property that return the RGB representation of the pola params """
        (inten, aop, dop) = self.polarization
        hsv = np.uint8(cv2.merge(((aop+np.pi/2)/np.pi*180,
                                  dop*255,
                                  inten/inten.max()*255)))
        rgb = cv2.cvtColor(hsv, cv2.cv.CV_HSV2BGR)
        return rgb

    def __repr__(self):
        """Representation function of the Polaim class"""
        return "Polacam image ({})".format(self.method)

    @property
    def export(self):
        """Image representation for export"""
        (inten, aop, dop) = self.polarization
        nbr, nbc = self.rgb.shape[0], self.rgb.shape[1]
        fina = np.zeros((nbr*2, nbc*2, 3), dtype='uint8')
        aop_colorHSV = np.uint8(cv2.merge(((aop+np.pi/2)/np.pi*180,
                                           np.ones(aop.shape)*255,
                                           np.ones(aop.shape)*255)))
        aop_colorRGB = cv2.cvtColor(aop_colorHSV, cv2.cv.CV_HSV2BGR)

        for c in range(3):
            fina[:nbr, :nbc, c] = np.uint8(inten/inten.max()*255)
            fina[:nbr, nbc:, c] = aop_colorRGB[:, :, c]
            fina[nbr:, :nbc, c] = np.uint8(dop*255)
            fina[nbr:, nbc:, c] = self.rgb[:, :, c]
        return fina


    @property
    def stokesParams(self):
        """Property that retruns an image with the first three Stokes parameters
        : I as the summation of all, Q as the substract of I0 and I90 and U as
        the substract of I45 and I 135 and last one I as I0 + I90 only """

        Js = self.images
        nbr, nbc = self.rgb.shape[0], self.rgb.shape[1]
        # Stokes = np.zeros((nbr*2, nbc*2,3), dtype='uint8')
        # for c in range(3):
        #     Stokes[:nbr, :nbc, c] = Js[0] + Js[1] + Js[2] + Js[3]
        #     Stokes[:nbr, nbc:, c] = Js[0] - Js[2]
        #     Stokes[nbr:, :nbc, c] = Js[1] - Js[3]
        #     Stokes[nbr:, nbc:, c] = Js[0] + Js[2]

        Stokes = np.zeros((nbr*2, nbc*2), dtype='uint8')
        Stokes[:nbr, :nbc] = Js[0] + Js[1] + Js[2] + Js[3]
        Stokes[:nbr, nbc:] = Js[0] - Js[2]
        Stokes[nbr:, :nbc] = Js[1] - Js[3]
        Stokes[nbr:, nbc:] = Js[0] + Js[2]

        return Stokes


def _polar_image(imCAM, method):
    imp = polar_image(imCAM, method)
    return imp


class polarized_analysis():
    def __init__(self):
        # Initialize the node for analysis of the images
        rospy.init_node('polarized_analysis', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # publishing to the topic stokes_params and Polar_params
        self.stokes_pub = rospy.Publisher("Stokes_params", Image,
                                          queue_size=5)
        self.polar_pub = rospy.Publisher("Polar_params", Image, queue_size=5)

        # Suscribe to topic camera/image to get the data
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/pleora_polarcam/image_raw",1, Image, self.callback)

        rospy.spin()

    def callback(self, data):
        # importing from sensor_msgs.Image to cv_image
        cv_image = self.bridge.imgmsg_to_cv2(data)
      
        # using polar_image class to process the image using ratliff3
        # interpolartion
        imp = _polar_image(cv_image, 'superpixel')
        # publisging the processed Stokes params and the exported images

        # Showsing the original image and the processed image

        cv2.imshow('Polarized parameters', imp.export)
        cv2.waitKey(1)
        cv2.imshow('Stokes images', imp.stokesParams)
        cv2.waitKey(1)
        self.polar_pub.publish(self.bridge.cv2_to_imgmsg(imp.export))
        self.stokes_pub.publish(self.bridge.cv2_to_imgmsg(imp.stokesParams))

    def shutdown(self):
        rospy.loginfo("Stopping the node ...")
        rospy.sleep(1)

# def main(args):
#     PA = polarized_analysis()
#     rospy.init_node('polarized_analyzes', anonymous=False)

#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#         cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        polarized_analysis()
    except:
        rospy.loginfo("polarized_analysis node terminated")
