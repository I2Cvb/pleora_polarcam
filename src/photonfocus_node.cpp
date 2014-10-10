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
//    dynamic_reconfigure::Server<gige_camera::FlirConfig> reconfig_svr_;

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
//        reconfig_svr_.setCallback(boost::bind(&PhotonFocusNode::configCb, this, _1, _2));
        std::cout << "===== PhotonFocus Camera ----- START ===== " << std::endl;
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

// TODO parameter server configure callback
//    void configCb(gige_camera::FlirConfig &config, uint32_t level)
//    {
//        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->stop();

//        cam_->setIRFormat(config.IRFormat);
//        if (config.AutoFocus) cam_->autoFocus();
//        //cam_->setSize(config.Width, config.Height);

//        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->start();
//    }

};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"photonfocus_node");
    ros::NodeHandle node_handle("~");

    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <MAC_ADDRESS | IP_ADDRESS>" << std::endl;
        return 0;
    }

    PhotonFocusNode camera_handle(argv[1],node_handle);
    ros::spin();
    return 0;
}
