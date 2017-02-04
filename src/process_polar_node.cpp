#define BOOST_LOG_DYN_LINK 1
#include <boost/scoped_ptr.hpp>
#include <boost/log/trivial.hpp>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <signal.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <boost/filesystem.hpp>
#include <std_msgs/String.h>


#include <pix2image.h>


void processCallback(const sensor_msgs::ImageConstPtr& msg,
                     ros::NodeHandle &node_handle){

    // parsed image from original image

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        //cv::imshow("Output image", cv_bridge::toCvCopy(msg, "mono8")-> image);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());

    }
    /* Add any OpenCV processing here */
    /* Gray scale image */
    cv::Mat img;
    img = cv_ptr->image.clone();
    
   
    // //std::vector<cv::Mat> angle_image = POLPro::raw2mat(img, false);
    std::vector<cv::Mat> output_img = POLPro::compute_polar_params
        (img, false);
    POLPro::imshow(output_img, false, false); 
 
}

bool ros_shutdown = false;


int main( int argc, char** argv )
{
    // initializing the ros node
    ros::init(argc, argv, "polars");
    // creating ros handle, main access point to communicate to ros
    ros::NodeHandle nh;

 
    cv::namedWindow("Output image");
    cv::startWindowThread();
    image_transport::ImageTransport n(nh);

    // suscribing to image_raw topic 
    image_transport::Subscriber sub = n.subscribe("pleora_polarcam_driver/image_raw"
                                                  , 1,
                                                  boost::bind(processCallback,
                                                              _1, 
                                                              boost::ref(nh)));
    
    // ros::spin();
    // cv::destroyWindow("Output image");


    while(ros::ok() && !ros_shutdown)
        ros::spinOnce();


    ros::shutdown();

    //ros::spin();
    return 0 ;

}

