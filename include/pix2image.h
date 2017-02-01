#ifndef pix2image
#define pix2image

#include <vector>

//#include <iomainip>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/scoped_ptr.hpp>



//#include <photonfocus_camera.h>

namespace POLPro
{
    std::vector<cv::Mat> raw2mat(const cv::Mat& origin, bool show);

    std::vector<cv::Mat> compute_stokes(const cv::Mat& origin, bool show);
    std::vector<cv::Mat> compute_stokes(
        const std::vector<cv::Mat>& angles_img, bool show);

    std::vector<cv::Mat> compute_polar_params(const cv::Mat& origin, bool show);
    std::vector<cv::Mat> compute_polar_params(
        const std::vector<cv::Mat>& origin, bool show);

    void imshow(std::vector<cv::Mat> img, bool as_hsv, bool as_stokes);
    int minmax(cv::Mat img, std::string s);

    //ros functions
    void processCallback(const sensor_msgs::ImageConstPtr& msg); 

}
#endif //pix2image.h
