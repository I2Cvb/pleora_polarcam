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
//#include <ira_photonfocus_driver/photonfocusConfig.h>

#include <pix2image.h>
//#include <photonfocus_camera.h>

namespace POLPro
{
    std::vector<cv::Mat> raw2mat(const cv::Mat& origin, const bool show=true) {
        // define the size of the output
        cv::Size output_size(origin.cols / 2, origin.rows / 2);
        // declare the vector containing the 4 angles images
        const int nb_angles = 4;
        std::vector<cv::Mat> output_img(nb_angles);
        for (auto it = output_img.begin(); it != output_img.end(); ++it)
            *it = cv::Mat::zeros(output_size, CV_8U);

        // copy the data in the new image
        for (int angle = 0; angle < nb_angles; ++angle) {
            int offset_row = angle / 2;
            int offset_col = angle % 2;
            BOOST_LOG_TRIVIAL(debug) << "offset_row " << offset_row
                                     << " offset_col " << offset_col;

            for (int row = 0; row < origin.rows/2; ++row)
                for (int col = 0; col < origin.cols/2; ++col)
                    output_img[angle].at<uchar>(row, col) = origin.at<uchar>(
                        2 * row + offset_row, 2 * col + offset_col);
        }

        if (show)
            imshow(output_img, false, false);

        return output_img;
    }

    std::vector<cv::Mat> compute_stokes(const std::vector<cv::Mat>& angles_img,
                                        const bool show=true) {
        // define the number of images to have for Stokes
        const int nb_stokes_img = 3;
        // Create zeros images
        std::vector<cv::Mat> output_img(nb_stokes_img);
        for (auto it = output_img.begin(); it != output_img.end(); ++it)
            *it = cv::Mat::zeros(angles_img[0].size(), CV_32F);

        // compute the Stokes parameters maps
        // S0: add the different angles
        for (auto it = angles_img.begin(); it != angles_img.end(); ++it)
            cv::add(output_img[0], *it, output_img[0], cv::noArray(),
                    CV_32F);
        output_img[0] /= 2.0;
        BOOST_LOG_TRIVIAL(debug) << minmax(output_img[0], "s0");

        // S1: subtract angles 0 and 90
        cv::subtract(angles_img[0], angles_img[2], output_img[1],
                     cv::noArray(), CV_32F);
        BOOST_LOG_TRIVIAL(debug) << minmax(output_img[1], "s1");

        // S2: subtract angles 45 and 135
        cv::subtract(angles_img[1], angles_img[3], output_img[2],
                     cv::noArray(), CV_32F);
        BOOST_LOG_TRIVIAL(debug) << minmax(output_img[2], "s2");

        if (show)
            imshow(output_img, false, true);

        return output_img;
    }

    std::vector<cv::Mat> compute_stokes(const cv::Mat& origin,
                                        const bool show=true) {
        // refactor the raw image
        std::vector<cv::Mat> angles_img = raw2mat(origin, show);

        return compute_stokes(angles_img, show);
    }

    std::vector<cv::Mat> compute_polar_params(
        const std::vector<cv::Mat>& origin, const bool show=true) {
        std::vector<cv::Mat> stokes_img;
        // Check if we have the original data or the stokes
        if (origin.size() == 4) {
            stokes_img = compute_stokes(origin, show);
        } else {
            stokes_img = origin;
        }

        // define the number of maps
        const int nb_params = 3;
        // create the zeros images
        std::vector<cv::Mat> output_img(nb_params);
        for (auto it = output_img.begin(); it != output_img.end(); ++it)
            *it = cv::Mat::zeros(stokes_img[0].size(), CV_32F);

        // compute the polar coordinate in degrees
        cv::cartToPolar(stokes_img[1], stokes_img[2],
                        output_img[0], output_img[1],
                        true);
        // normalize the maps
        // degree of polarization
        output_img[0] /= stokes_img[0];
        // angle of polarization
        output_img[1] *= 0.5;
        // copy s0
        stokes_img[0].copyTo(output_img[2]);
        if (show)
            imshow(output_img, false, false);

        return output_img;
    }

    std::vector<cv::Mat> compute_polar_params(const cv::Mat& origin,
                                              const bool show=true) {
        // compute the Stokes' parameters
        std::vector<cv::Mat> stokes_img = compute_stokes(origin, show);

        return compute_polar_params(stokes_img, show);
    }

    std::string minmax(const cv::Mat& img, const std::string& s) {
        double min, max;
        cv::Point idmin, idmax;
        cv::minMaxLoc(img, &min, &max, &idmin, &idmax) ;

        return "Image " + s
            + ": min=" + std::to_string(min)
            + " - max= " + std::to_string(max);
    }

    void imshow(std::vector<cv::Mat> img, const bool as_hsv=false,
                const bool is_stokes=true) {

        // through an error if there is not 3d img and hsv is turned on
        if ((img.size() != 3) && as_hsv)
            throw std::invalid_argument("img needs to be a 3 channels images"
                                        " if you need hsv support");

        // Convert the data if Stokes or polarization parameters
        if (img.size() == 3) {
            if (is_stokes) {
                // Stokes parameters normalization
                img[0] /= 2.0;
                img[1] = (img[1] + 255.0) / 2.0;
                img[2] = (img[2] + 255.0) / 2.0;
            } else {
                // polarization parameters normalization
                img[0] = img[0] * 255;
                img[2] = img[2] / 2;
            }
            // Convert to uint8
            // for (auto it = img.begin(); it != img.end(); ++it)
            //     *it.convertTo(*it, CV_8UC1);
            for (int i = 0; i < img.size(); ++i)
                img[i].convertTo(img[i], CV_8UC1);
        }

        // Declare the output image
        cv::Mat output_img;

        if (as_hsv) {
            // Merge the image together to have a 3 channels image

            std::vector<cv::Mat> channels;
            channels.push_back(img[1].clone()); 
            channels.push_back(img[0].clone()); 
            channels.push_back(img[2].clone());

            cv::Mat bgr_img;
            cv::merge(channels, bgr_img);
            cv::cvtColor(bgr_img, output_img, CV_HLS2BGR);
        } else {
            // Concatenate the images available together
            cv::Size img_size(img[0].cols, img[0].rows);
            output_img = cv::Mat::zeros
                (img[0].rows*2, img[0].cols*2, CV_8UC1);
            int rows = img[0].rows; 
            int cols = img[0].cols;
            
    
            for (int i = 0; i < img.size(); ++i) {
                // we need to shift the image next to each other properly
                int offset_col = i % 2;
                int offset_row = i / 2;
               
                img[i].copyTo(output_img(
                                  cv::Rect(img_size.width * offset_col,
                                           img_size.height * offset_row,
                                           img_size.width,
                                           img_size.height)));

            }
        }

        cv::imshow("Output image", output_img);
        cv::waitKey(0);
    }
}  // Namespace POLPro

void processCallback(const sensor_msgs::ImageConstPtr& msg){

    // parsed image from original image

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        cv::imshow("view", cv_bridge::toCvCopy(msg, "mono8")-> image);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());

    }


    /* Add any OpenCV processing here */
    /* Gray scale image */
    //cv::Mat img;
    //img = cv_ptr->image.clone();
    //std::vector<cv::Mat> angle_image = POLPro::raw2mat(img, true);

    //cv::imshow("view", angle_image[0]);
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



    // cv::namedWindow("parsed Image");
    // cv::startWindowThread();
    image_transport::ImageTransport n(nh);
    image_transport::Subscriber sub = n.subscribe("pleora_polarcam/image_raw"
                                                  , 1, processCallback);

    ros::spin();
    // cv::destroyWindow("parsed Image");


    ros::spin();
    return 0 ;

}

//---------------------------------------------//
// int main(int argc, char** argv) {
//    if (argc != 2) {
//        std::cout <<" Usage: display_image ImageToLoadAndDisplay" << std::endl;
//        return -1;
//    }

//    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

//    if (!image.data) {
//        std::cout <<  "Could not open or find the image" << std::endl;
//        return -1;
//    }

//    // parsed image from original image
//    std::vector<cv::Mat> angle_image = POLPro::raw2mat(image, true);

//    // Stokes parameters
//    std::vector<cv::Mat> stokes_images = POLPro::compute_stokes(angle_image, 
//                                                                false);

//    // polar components
//    std::vector<cv::Mat> polar_images =
//        POLPro::compute_polar_params(stokes_images, true);

//    //POLPro::imshow(angle_image);
//    // POLPro::imshow(stokes_images); 
//    //POLPro::imshow(polar_images, false, false); 
//   return 0;
// }
