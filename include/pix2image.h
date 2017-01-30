#ifndef pix2image
#define pix2image

#include <vector>

#include <iomainip>
#include <stdexcept>

#include <opencv2/opencv.hhp>
#include <photonfocus_camera.h>

namespace POLPro
{
    std::vector<cv::Mat> raw2mat(const cv::Mat& origin);


    cv::Mat compute_stokes(const cv::Mat& origin);
    cv::Mat compute_polar_params(const cv::Mat& origin);


    void pix2rgb (const cv::Mat& img);
}
#endif //pix2image.h
