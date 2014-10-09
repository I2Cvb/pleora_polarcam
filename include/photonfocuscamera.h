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
#include <stdexcept>
#include <vector>
#include <assert.h>

#include <boost/function.hpp>

#include "PvDeviceGEV.h"
#include "PvStreamGEV.h"

namespace IRALab
{
namespace PhotonFocus
{
class Camera
{
    PvDeviceGEV * device;
    PvString camera_id; // MAC or IP
    PvStreamGEV * stream;

    PvGenParameterArray * device_parameters;
    PvGenParameterArray * stream_parameters;

    std::vector<PvBuffer *> buffer;

    boost::function<void (const PvBuffer *)> callback;

public:
    Camera(std::string ip_address);
    ~Camera();
    void start();
private:
    PvResult openStream();
    void initBuffer();
    void freeBuffer();

    void setFrameCallback(boost::function<void (const PvBuffer *)> callback);

    void setRoiToWholeFrame();

    PvAccessType getAccessType();
    long getAttribute(std::string name, long * min = NULL, long * max = NULL);
    void setAttribute(std::string name, long value);
};
}
}
#endif // PHOTONFOCUSCAMERA_H
