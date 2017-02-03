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
                     const std::string& s, ros::NodeHandle &node_handle){

    // parsed image from original image

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        //cv::imshow("view", cv_bridge::toCvCopy(msg, "mono8")-> image);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());

    }
    /* Add any OpenCV processing here */
    /* Gray scale image */
    cv::Mat img;
    img = cv_ptr->image.clone();
    
    if (s == "stokes"){

        //std::vector<cv::Mat> angle_image = POLPro::raw2mat(img, false);
        std::vector<cv::Mat> output_img = POLPro::compute_stokes
            (img, false);
        POLPro::imshow(output_img, false, true); 
        
    }else if (s == "polar"){
        std::vector<cv::Mat> output_img = POLPro::compute_polar_params
            (img, false);
        POLPro::imshow(output_img, false, false); 
    }else{
         std::vector<cv::Mat> output_img = POLPro::raw2mat(img, true);
         POLPro::imshow(output_img, false, false); 

         // //publishing array of images
         // image_transport::Publisher pub = node_handle.advertise
         //     <Static_Image_Publisher::ArrayImages>("/"+ s, 1, true); 
 
         // sensor_msgs::CvBridge bridge_;
         // Static_Image_Publisher::ArrayImages rosimgs;
         // for (int i = 0; i < output_img.size(); ++i) {
         //     output_img[i].convertTo(output_img[i], CV_8UC1);

         //     try {
         //         rosimgs.data.push_back(*(bridge_.cvToImgMsg(output_img[i], 
         //                                                     "mono8")));
         //     } catch (sensor_msgs::CvBridgeException error) {
         //         ROS_ERROR("error");
         //     }
         // }
         // pub.publish(rosimgs);     // publishing array of images 

         
    }

}


int main( int argc, char** argv )
{
    // initializing the ros node
    ros::init(argc, argv, "pix2image");
    // creating ros handle, main access point to communicate to ros
    ros::NodeHandle nh;

    // suscribing to the pleora_polarcam node and topic raw_image
    //ros::Subscriber sub = n.subscribe("pleora_polarcam/raw_image",
    //				      1000, processCallback);

    if (argc != 2) {
        std::cout <<" Usage: display_image Option (mat, stokes, polar)" 
                  << std::endl;
        return -1;
    }
    std::string s = argv[1]; 
    cv::namedWindow("Output image");
    cv::startWindowThread();
    image_transport::ImageTransport n(nh);

    // suscribing to image_raw topic 
    image_transport::Subscriber sub = n.subscribe("pleora_polarcam/image_raw"
                                                  , 1,
                                                  boost::bind(processCallback,
                                                              _1, s, 
                                                              boost::ref(nh)));
    
    ros::spin();
    cv::destroyWindow("Output image");


    ros::spin();
    return 0 ;

}

