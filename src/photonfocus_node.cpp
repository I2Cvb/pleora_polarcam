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

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <photonfocuscamera.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <photonfocus_camera/photonfocusConfig.h>

class PhotonFocusNode
{
private:
    // ROS
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport;
    image_transport::Publisher publisher;

    // ROS Message
    sensor_msgs::ImagePtr image;

    // FLIR Camera
    boost::scoped_ptr<IRALab::PhotonFocus::Camera> camera;

    // TODO Dynamic Reconfigure [with parameter server]
    dynamic_reconfigure::Server<photonfocus_camera::photonfocusConfig> reconfig_svr_;

public:
    PhotonFocusNode(std::string ip,const ros::NodeHandle & node_handle):
        node_handle(node_handle),
        image_transport(node_handle),
        publisher(image_transport.advertise("image_raw",1)),
        camera(NULL)
    {
        camera.reset(new IRALab::PhotonFocus::Camera(ip));
        camera->start();
        camera->callback = boost::bind(&PhotonFocusNode::publishImage, this, _1);
        // TODO parameter server callback
        reconfig_svr_.setCallback(boost::bind(&PhotonFocusNode::configCb, this, _1, _2));
        std::cout << std::setw(80) << std::setfill(' ') << std::left << "===== PhotonFocus Camera ----- START ===== " << std::endl;
    }

    ~PhotonFocusNode()
    {
        camera->stop();
        camera.reset();
        std::cout << "===== PhotonFocus Camera ----- STOP  ===== " << std::endl;
    }

    void publishImage(const cv::Mat img)
    {
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = img;
        this->image = cv_image.toImageMsg();
        publisher.publish(this->image);
    }

    void configCb(photonfocus_camera::photonfocusConfig & config, uint32_t level)
    {
        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera->stop();

        //# ----- Image Size Control -----
        camera->setDeviceAttribute<PvGenInteger,long>("Width",config.Width*32+768);
        camera->setDeviceAttribute<PvGenInteger,long>("Height",config.Height);

        camera->setDeviceAttribute<PvGenInteger,long>("OffsetX",config.OffsetX*32);
        camera->setDeviceAttribute<PvGenInteger,long>("OffsetY",config.OffsetY);

        //# ----- Exposure and FrameRate -----
        camera->setDeviceAttribute<PvGenFloat,double>("ExposureTimeAbs",config.ExposureTimeAbs);
        camera->setDeviceAttribute<PvGenBoolean,bool>("ConstantFramerate_CFR",config.ConstantFramerate_CFR);
        if(config.ConstantFramerate_CFR)
            camera->setDeviceAttribute<PvGenFloat,double>("Frametime",config.Frametime);

        camera->setDeviceAttribute<PvGenBoolean,bool>("Trigger_Interleave",config.Trigger_Interleave);
        if(!config.Trigger_Interleave)\
        {
            camera->setDeviceAttribute<PvGenEnum,long>("LinLog_Mode",config.LinLog_Mode);
            if(config.LinLog_Mode == 4)
            {
                std::cout << "UserDefined" << std::endl;
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value1",config.LinLog_Value1);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value2",config.LinLog_Value2);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time1",config.LinLog_Time1);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time2",config.LinLog_Time2);
            }
        }
        camera->setDeviceAttribute<PvGenInteger,long>("Voltages_BlackLevelOffset",config.Voltages_BlackLevelOffset);

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera->start();
    }

};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"photonfocus_node");
    ros::NodeHandle node_handle("~");

    std::string ip;

    if(!node_handle.getParam("ip",ip))
    {
        std::cout << "Usage: " << argv[0] << " _ip:=IP_ADDRESS" << std::endl;
        return 0;
    }

    PhotonFocusNode camera_handle(ip,node_handle);
    ros::spin();
    return 0;
}
