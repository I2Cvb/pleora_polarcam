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

#include "photonfocuscamera.h"

// THIS MACRO EXPLOITS RESULT DESCRIPTION FOR THROWING EXCEPTIONS ON EXPR WHICH RETURNS A PvResult
#define CHECK_RESULT(expression)\
do{\
    PvResult result = expression;\
    if(!result.IsOK())\
        throw std::runtime_error(result.GetDescription().GetAscii());\
}while(false)

namespace IRALab
{
namespace PhotonFocus
{
Camera::Camera(std::string ip_address)
    : camera_id(ip_address.c_str())
{
    // Find the camera and connect to it
    PvAccessType access_type = getAccessType();
    if(access_type == PvAccessOpen) // The device is reachable and no one is connected to it
    {
        PvResult result;
        device = static_cast<PvDeviceGEV *>(PvDevice::CreateAndConnect(camera_id,&result));
        if(device == NULL || !result.IsOK()) // for some reason the device is not reachable anymore...
            throw std::runtime_error("Device " + ip_address + " not found!"); // TODO custom exeception
    }
    else if(access_type == PvAccessUnknown) // the device is not reachable
        throw std::runtime_error("Device " + ip_address + " not found!"); // TODO custom exeception
    else // the device is reachable, but someone is connected to it
        throw std::runtime_error("Another process is using the camera " + ip_address); // TODO custom exeception

    device_parameters = device->GetParameters();

    setRoiToWholeFrame();
    assert(device != NULL);
    openStream(); // TODO std::string("Unable to stream from ") + camera_id.GetAscii()
    assert(stream != NULL);
    initBuffer();
}


Camera::~Camera()
{
    // Disable streaming on the device
    device->StreamDisable();

    // Abort all buffers from the stream and dequeue
    stream->AbortQueuedBuffers();

    while(stream->GetQueuedBufferCount() > 0)
    {
        PvBuffer *frame = NULL;
        PvResult result;

        stream->RetrieveBuffer( &frame, &result );
    }

    freeBuffer();

    CHECK_RESULT(stream->Close());
    PvStream::Free(stream);

    CHECK_RESULT(device->Disconnect());
    PvDevice::Free(device);
}

PvResult Camera::openStream()
{
    PvDeviceGEV * device = static_cast<PvDeviceGEV *>(this->device);
    CHECK_RESULT(device->NegotiatePacketSize());

    PvResult result;
    stream = static_cast<PvStreamGEV *>(PvStream::CreateAndOpen(camera_id,&result));
    if(stream == NULL)
        throw std::runtime_error(std::string("Unable to stream from ") + camera_id.GetAscii() + ".");

    // Configure device streaming destination
    device->SetStreamDestination(stream->GetLocalIPAddress(), stream->GetLocalPort());
}

void Camera::initBuffer()
{
    int payload_size = device->GetPayloadSize();

    // Use BUFFER_COUNT or the maximum number of buffers, whichever is smaller
    int buffer_length = (stream->GetQueuedBufferMaximum() < BUFFER_COUNT) ? stream->GetQueuedBufferMaximum() : BUFFER_COUNT;

    // allocate frame buffers
    for(int i=0;i < buffer_length;i++)
    {
        PvBuffer * frame = new PvBuffer;
        CHECK_RESULT(frame->Alloc(payload_size));
        buffer.push_back(frame);
    }

    // Queue all buffers in the stream
    std::vector<PvBuffer *>::iterator frame = buffer.begin();
    while(frame != buffer.end())
    {
        stream->QueueBuffer(*frame);
        frame++;
    }
}

void Camera::freeBuffer()
{
    std::vector<PvBuffer *>::iterator frame = buffer.begin();
    while(frame != buffer.end())
    {
        delete *frame;
        frame++;
    }
    buffer.clear();
}

void Camera::setFrameCallback(boost::function<void (const PvBuffer *)> callback)
{
    this->callback = callback;
}

void Camera::setRoiToWholeFrame()
{
    long value;
    long max_width;
    value = getAttribute("Width",NULL,&max_width);
    setAttribute("Width",max_width);

    long max_height;
    value = getAttribute("Height",NULL,&max_height);
    setAttribute("Height",max_height);
}

PvAccessType Camera::getAccessType()
{
    PvAccessType access_type;
    PvDeviceGEV::GetAccessType(camera_id,access_type);
    return access_type;
}

long Camera::getAttribute(std::string name, long *min, long *max)
{
    PvGenInteger * parameter = dynamic_cast<PvGenInteger *>(device->GetParameters()->Get(PvString(name.c_str())));

    if(parameter == NULL)
        throw std::runtime_error("Attribute " + name + " does not exist.");

    long value;
    CHECK_RESULT(parameter->GetValue(value));
    if(min != NULL)
        CHECK_RESULT(parameter->GetMin(*min));
    if(max != NULL)
        CHECK_RESULT(parameter->GetMax(*max));
    return value;
}

void Camera::setAttribute(std::string name, long value)
{
    PvGenInteger * parameter = dynamic_cast<PvGenInteger *>(device->GetParameters()->Get(PvString(name.c_str())));

    if(parameter == NULL)
        throw std::runtime_error("Attribute " + name + " does not exist.");

    CHECK_RESULT(parameter->SetValue(value));
}

}
}
