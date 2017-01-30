#ifndef pix2image
#define pix2image

#include <vector>

//#include <iomainip>
#include <stdexcept>

#include <opencv2/opencv.hpp>
//#include <photonfocus_camera.h>

namespace POLPro
{
    std::vector<cv::Mat> raw2mat(const cv::Mat& origin);

    std::vector<cv::Mat> compute_stokes(const cv::Mat& origin);
    std::vector<cv::Mat> compute_stokes(
        const std::vector<cv::Mat>& angles_img);

    std::vector<cv::Mat> compute_polar_params(const cv::Mat& origin);
    std::vector<cv::Mat> compute_polar_params(
        const std::vector<cv::Mat>& origin);


    //  void pix2rgb (const cv::Mat& img);
}
#endif //pix2image.h
