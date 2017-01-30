#ifndef pix2image
#define pix2image

#include <vector>

#include <iomainip>
#include <stdexcept>

#include <opencv2/opencv.hhp>
#include <photonfocus_camera.h>

namespace POLPro
{
    /**
       @brief Convert the raw data to opencv image

       @param Mat origin Raw image extracted from the polar cam.

    **/
    cv::Mat raw2mat1d(const cv::Mat& origin);
    std::vector<cv::Mat> raw2mat4d(const cv::Mat& origin);


    cv::Mat compute_stokes(const cv::Mat& origin);
    cv::Mat compute_polar_params(const cv::Mat& origin);


    void pix2rgb (const cv::Mat& img);
}
#endif //pix2image.h
