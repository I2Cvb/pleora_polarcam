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

#include "photonfocus_camera.h"

namespace IRALab
{
PhotonFocusCamera::PhotonFocusCamera(std::string ip_address)
    : camera_id(ip_address.c_str())
{
    // Find the camera and connect to it
    PvAccessType access_type = getAccessType();
    if(access_type == PvAccessOpen) // The device is reachable and no one is connected to it
    {
        PvResult result;
        //PvStream *lStream;
        std::cout << "Opening stream to device." << std::endl;
        //lStream = PvStream::CreateAndOpen(camera_id,&result);
        device = static_cast<PvDeviceGEV *>(PvDevice::CreateAndConnect(camera_id,&result));
        if(device == NULL || !result.IsOK()) // for some reason the device is not reachable anymore...
            throw std::runtime_error("Device " + ip_address + " not found!"); // TODO custom exeception
    }
    else if(access_type == PvAccessUnknown) // the device is not reachable
        throw std::runtime_error("Device " + ip_address + " not found!"); // TODO custom exeception
    else // the device is reachable, but someone is connected to it
        throw std::runtime_error("Another process is using the camera " + ip_address); // TODO custom exeception

    device_parameters = device->GetParameters();
}


PhotonFocusCamera::~PhotonFocusCamera()
{
    CHECK_RESULT(device->Disconnect());
    PvDevice::Free(device);
    std::cout << std::endl;
}

void PhotonFocusCamera::start()
{
    open();

    //    // TLParamsLocked is optional but when present, it MUST be set to 1
    //    // before sending the AcquisitionStart command
    //    device_params_->SetIntegerValue( "TLParamsLocked", 1 );

    device_parameters->ExecuteCommand( "GevTimestampControlReset" );

    // All is set and ready, now say to the camera to start sending images
    device_parameters->ExecuteCommand("AcquisitionStart");

    // Start the thread which polls images from the camera buffer
    image_thread.reset(new boost::thread(boost::bind(&IRALab::PhotonFocusCamera::acquireImages, this)));
}

void PhotonFocusCamera::stop()
{
    // Tell the camera to stop sending images
    device_parameters->ExecuteCommand( "AcquisitionStop" );
    close();
}

void PhotonFocusCamera::open()
{
    // Test the network connection for the largest possible packet size that the network can support on the link between camera and controller
    PvDeviceGEV * device = static_cast<PvDeviceGEV *>(this->device);
    CHECK_RESULT(device->NegotiatePacketSize());

    // Open a stream with the device
    PvResult result;
    stream = PvStream::CreateAndOpen(camera_id,&result);
    if(stream == NULL)
        throw std::runtime_error(std::string("Unable to stream from ") + camera_id.GetAscii() + ".");

    stream_parameters = stream->GetParameters(); // get stream parameters (for future usages)

    // One endpoint of the stream is the camera itself, the other one is this software on the controller and it must be specified
    PvStreamGEV * stream_gev = static_cast<PvStreamGEV *>(this->stream);
    device->SetStreamDestination(stream_gev->GetLocalIPAddress(), stream_gev->GetLocalPort());

    // Pipeline initialization (it manages buffers)
    pipeline = new PvPipeline(stream);

    int payload_size = device->GetPayloadSize();

    // Set the Buffer count and the Buffer size
    pipeline->SetBufferCount(BUFFER_COUNT);
    pipeline->SetBufferSize(payload_size);

    // IMPORTANT: the pipeline needs to be "armed", or started before we instruct the device to send us images
    pipeline->Start();
    device->StreamEnable();
}

void PhotonFocusCamera::close()
{
    // STOP THE THREAD WHICH RETRIEVE IMAGES FROM THE CAMERA AND WAIT FOR ITS CONCLUSION!
    image_thread->interrupt();
    image_thread->join(); // TODO is it necessary?
    image_thread.reset();

    device->StreamDisable();

    pipeline->Stop();
    delete pipeline;

    stream->Close();
    PvStream::Free(stream);
}

void PhotonFocusCamera::acquireImages()
{
    char doodle[] = "|\\-|-/";
    int doodle_index = 0;

    double framerate = 0.0;
    double bandwidth = 0.0;
    long error_count = 0;
    long image_average = 0;
    PvString last_error;

    while(true)
    {
        PvBuffer *buffer = NULL;
        PvImage *image = NULL;
        cv::Mat raw_image;

        PvResult buffer_result, operation_result;

        operation_result = pipeline->RetrieveNextBuffer(&buffer, 1000, &buffer_result);

        if(operation_result.IsOK()) // operation results says about the retrieving from the pipeline
        {
            if(buffer_result.IsOK()) // buffer results says about the retrieved buffer status
            {
                CHECK_RESULT(stream_parameters->GetFloatValue("AcquisitionRate", framerate));
                CHECK_RESULT(stream_parameters->GetFloatValue("Bandwidth", bandwidth));
                CHECK_RESULT(stream_parameters->GetIntegerValue("ErrorCount",error_count));
                CHECK_RESULT(stream_parameters->GetEnumValue("LastError",last_error));
//                CHECK_RESULT(device_parameters->GetIntegerValue("Average_Value",image_average));

                std::cout << std::fixed << std::setprecision(1);
                std::cout << doodle[doodle_index];
                if(buffer->GetPayloadType() == PvPayloadTypeImage)
                {
                    image = buffer->GetImage();
                    raw_image = cv::Mat(image->GetHeight(),image->GetWidth(),CV_8UC1,image->GetDataPointer());
                    //raw_image = cv::Mat(image->GetHeight(),image->GetWidth(),CV_16UC1,image->GetDataPointer());

                    std::cout << " W:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.cols
                              << " H:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.rows;

                    // !!!! THIS IS THE POINT WHERE THE EXTERNAL CALLBACK IS CALLED !!!!
                    callback(raw_image);
                }
                else
                    std::cout << " (buffer does not contain image)";

                std::cout << " "
                          << std::setw(3) << std::setfill(' ') << std::fixed << std::setprecision(1) << framerate << "FPS "
                          << std::setw(3) << (int)(bandwidth / 1000000.0) << "Mb/s"
                          << " AvgVal: " << std::setw(4) << std::setfill(' ') << std::right << image_average
                          << " * Errors:" << error_count << " - " << std::setw(26) << std::setfill(' ') << std::left << last_error.GetAscii() << "\r";
            }
            else{
                std::cout << doodle[doodle_index] << " " << buffer_result.GetCode() << " " << buffer_result.GetDescription().GetAscii() << "\r";
            }
            // release the buffer back to the pipeline
            pipeline->ReleaseBuffer(buffer);
        }
        else
        {
            std::cout << doodle[doodle_index] << " " << buffer_result.GetCode() << " " << buffer_result.GetDescription().GetAscii() << "\r";
        }
        // when the interruption on the thread is called, its execution must reach this point! in this way the whole should be in a clear state
        boost::this_thread::interruption_point();
        ++doodle_index %= 6;
    }
}

PvAccessType PhotonFocusCamera::getAccessType()
{
    PvAccessType access_type;
    PvDeviceGEV::GetAccessType(camera_id,access_type);
    return access_type;
}

}
