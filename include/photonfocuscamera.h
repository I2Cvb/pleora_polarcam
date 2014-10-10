/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:	Alberto Invernizzi                       		           *
 *   Email:     alby.inve@gmail.com                                        *
 *   Date:      08/10/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef PHOTONFOCUSCAMERA_H
#define PHOTONFOCUSCAMERA_H

#define BUFFER_COUNT 16

#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>

#include "PvDeviceGEV.h"
#include "PvStreamGEV.h"
#include "PvPipeline.h"

namespace IRALab
{
namespace PhotonFocus
{
class Camera
{
    PvDevice * device;
    PvStream * stream;
    PvPipeline * pipeline;
    PvString camera_id; // MAC or IP

    PvGenParameterArray * device_parameters;
    PvGenParameterArray * stream_parameters;

    boost::shared_ptr<boost::thread> image_thread;

    cv::Size image_size;

public:
    boost::function<void(const cv::Mat &image)> callback;

    Camera(std::string ip_address);
    ~Camera();

    void start();
    void stop();
private:
    void open();
    void close();
    void acquireImages();

    void setRoiToWholeFrame();

    PvAccessType getAccessType();
    long getDeviceAttribute(std::string name, long * min = NULL, long * max = NULL);
    void setDeviceAttribute(std::string name, long value);
};
}
}
#endif // PHOTONFOCUSCAMERA_H
