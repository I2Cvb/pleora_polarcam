#ifndef pix2image
#define pix2image

#include <iomainip>
#include <stdexcept>

#include <opencv2/opencv.hhp>
#include <photonfocus_camera.h>

namespace POLPro
{
class Pix2Image
{
// I dont know what to add for private ?
 
public:
    void pix2parsed (const cv::Mat img); 
    void pix2rgb (const cv::Mat img);
}


}
#endif //pix2image.h
