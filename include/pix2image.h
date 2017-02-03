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
    std::vector<cv::Mat> raw2mat(const cv::Mat& origin, const bool show);

    std::vector<cv::Mat> compute_stokes(const cv::Mat& origin,
                                        const bool show);
    std::vector<cv::Mat> compute_stokes(
        const std::vector<cv::Mat>& angles_img, const bool show);

    std::vector<cv::Mat> compute_polar_params(const cv::Mat& origin,
                                              const bool show);
    std::vector<cv::Mat> compute_polar_params(
        const std::vector<cv::Mat>& origin, const bool show);

    void imshow(std::vector<cv::Mat> img, const bool as_hsv,
                const bool is_stokes);
    std::string minmax(const cv::Mat& img, const std::string& s);

    //ros functions
    void processCallback(const sensor_msgs::ImageConstPtr& msg, std::string& s, 
                         ros::NodeHandle &node_handle);

}
#endif //pix2image.h
