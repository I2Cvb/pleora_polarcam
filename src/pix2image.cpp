#include <pix2image.h>
//#include <photonfocus_camera.h>


#include <ros/ros.h>
#include <signal.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
//#include <ira_photonfocus_driver/photonfocusConfig.h>

#include <camera_info_manager/camera_info_manager.h>

#include <boost/scoped_ptr.hpp>




using namespace cv; 
using namespace std; 

namespace POLPro
{

    std::vector<cv::Mat> raw2mat(const cv::Mat& origin, bool show=true)
  {
        // define the size of the output
	cv::Size output_size(origin.cols / 2, origin.rows / 2);
	// declare the vector containing the 4 angles images
        const int nb_angles = 4;
	std::vector<cv::Mat> output_img(nb_angles);                           
        for (auto i = output_img.begin(); i != output_img.end(); ++i)
            *i = cv::Mat::zeros(output_size, CV_8U);

        // copy the data in the new image
        for (int angle = 0; angle < nb_angles; ++angle){
	    int offset_row = angle / 2;
	    int offset_col = angle % 2;
	    cout<< "offset_row " << offset_row << "  offset_col" << offset_col 
		<< std::endl; 

	    for (int row = 0; row < origin.rows/2; ++row){
                for (int col = 0; col < origin.cols/2; ++col){
		    output_img[angle].at<uchar>(row, col) = origin.at<uchar>(
                        2 * row + offset_row, 2 * col + offset_col); 
		}
	    }
	}		
	if(show)
	    imshow(output_img, false, false); 
        // Return the image
        return output_img;
    }

    std::vector<cv::Mat> compute_stokes(const std::vector<cv::Mat>& angles_img, 
					bool show=false)
    {
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
    	minmax(output_img[0], "s0");  
	

       // S1: subtract angles 0 and 90
	cv::subtract(angles_img[0], angles_img[2], output_img[1],
                     cv::noArray(), CV_32F);

	minmax(output_img[1], "s1"); 
	// S2: subtract angles 45 and 135
        cv::subtract(angles_img[1], angles_img[3], output_img[2],
                     cv::noArray(), CV_32F);
	minmax(output_img[2], "s2"); 
	
	if (!show)
	    imshow(output_img, false, true); 

        return output_img;
    }

    std::vector<cv::Mat> compute_stokes(const cv::Mat& origin, bool show=false)
    {
        // refactor the raw image
        std::vector<cv::Mat> angles_img = raw2mat(origin, show);

        return compute_stokes(angles_img, show);
    }

    std::vector<cv::Mat> compute_polar_params(
        const std::vector<cv::Mat>& origin, bool show=false)
    {
        std::vector<cv::Mat> stokes_img;
        // Check if we have the original data or the stokes
        if (origin.size() == 4)
        {
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
	// dop
        output_img[0] /= stokes_img[0];
        // aop
	output_img[1] *= 0.5;
        // copy s0
        stokes_img[0].copyTo(output_img[2]);
	if (!show)
	    imshow(output_img, true, false); 
        return output_img;
    }

    std::vector<cv::Mat> compute_polar_params(const cv::Mat& origin, 
					      bool show=false)
    {
        // compute the Stokes' parameters
        std::vector<cv::Mat> stokes_img = compute_stokes(origin, show);

        return compute_polar_params(stokes_img, show);
    }



    int minmax (cv::Mat img, std::string s)
    {
	double min, max;
	Point idmin, idmax;
	minMaxLoc(img, &min, &max, &idmin, &idmax) ;
        
	cout <<"min max " + s + " : " << min << " " << max << std::endl;
    }

    void imshow(std::vector<cv::Mat> img, bool as_hsv=false, 
		bool as_stokes=true)
    {
	int cols = img[0].rows; 
	int rows = img[0].cols; 
   
        // through an error if there is not 3d img and hsv is turned on
        if ((img.size() != 3) && as_hsv)
            throw std::invalid_argument("img needs to be a 3 channels images"
                                        " if you need hsv support");
	
        // Declarig the appropriate size depeding on the image size
	
	if ((img.size() == 3) && as_stokes && !as_hsv){

	    // create the zeros output image
	    cv::Mat output_img = cv::Mat::zeros
		(img[0].rows*2, img[0].cols*2, CV_8UC1);
	    
	    // conversion of CV_32F to CV_8UC1 
	    img[0] /= 2.0;
	    img[1] = (img[1]+255.0)/2.0; 
	    img[2] = (img[2]+255.0)/2.0; 
	    for (int i = 0;  i <=2; ++i){
		img[i].convertTo(img[i], CV_8UC1); 
	    }		
					       
	    // coping the data to the output image 
	    img[0].copyTo(output_img(cv::Rect(0, 0, 
					      img[0].cols, img[0].rows)));
	    img[1].copyTo(output_img(cv::Rect(rows, 0, 
					      img[1].cols, img[1].rows)));
	    img[2].copyTo(output_img(cv::Rect(0, cols, 
					      img[2].cols, img[2].rows)));
	    

	    imshow("Stokes-params", output_img); 


	}else if ((img.size() == 3) && !as_stokes){
	    
	   
	    // create the zeros output image
	    cv::Mat output_img = cv::Mat::zeros
		(img[0].rows*2, img[0].cols*2, CV_8UC1);
	    
	    // conversion from CV_32F tp CV_8UC1
	    img[0] = img[0]*255 ;
	    img[2] = img[2]/2; 
	    for (int i = 0; i <=2; ++i){
	       img[i].convertTo(img[i], CV_8UC1); 
	    }

            // coping the data to the output image 
	    img[0].copyTo(output_img(cv::Rect(0, 0, 
					      img[0].cols, img[0].rows)));
	    img[1].copyTo(output_img(cv::Rect(rows, 0, 
					      img[1].cols, img[1].rows)));
	    img[2].copyTo(output_img(cv::Rect(0, cols, 
					      img[2].cols, img[2].rows)));
	    

	    imshow("polar_params", output_img); 
	    

	    if (as_hsv){
		// merge the vector into a single matrix
		cv::Mat img_hsv; 
		merge(img, img_hsv); 

		// convert from bgr to hsl
		cv::Mat HSL; 
		cvtColor(img_hsv, HSL, CV_HLS2BGR);
    
		// show the hsv image 
		imshow("hsv image", img_hsv);  
	    
	    }else{
		  
		// DoP image 
		imshow("DoP", img[0]); 
		// AoP image
		imshow("AoP", img[1]); 
	    
	    }
        
    		 
	}else if (img.size() ==4){
	    // these images are already in 8 bit and does not require conversion
	    // create the zeros images
	    cv::Mat output_img(img[0].rows*2, img[0].cols*2, CV_8UC1);
	   
	    img[0].copyTo(output_img(cv::Rect(0, 0, 
					      img[0].cols, img[0].rows)));
	    img[1].copyTo(output_img(cv::Rect(rows, 0, 
					      img[1].cols, img[1].rows)));
	    img[2].copyTo(output_img(cv::Rect(0, cols, 
					      img[2].cols, img[2].rows)));
	    img[3].copyTo(output_img(cv::Rect(rows, cols, 
					      img[3].cols, img[3].rows)));


	    imshow("parsed_image", output_img); 
	    	    
	}else{
	    throw std::invalid_argument("img needs to be a 3 or 4 channels"); 
	}

	

    }
}


void processCallback(const sensor_msgs::ImageConstPtr& msg){

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
    std::vector<cv::Mat> angle_image = POLPro::raw2mat(img, false);
    
    cv::imshow("view", angle_image[0]);
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

//--------------------------------------------------//

// int main( int argc, char** argv )
// {
//     if( argc != 2)
//     {
// 	cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
//      return -1;
//     }

//     Mat image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

//     if(! image.data )                              // Check for invalid input
//     {
//         cout <<  "Could not open or find the image" << std::endl ;
//         return -1;
//     }

//     // parsed image from original image 
//     std::vector<cv::Mat> angle_image = POLPro::raw2mat(image, true); 
    
//     // Stokes parameters 
//     //std::vector<cv::Mat> stokes_images = POLPro::compute_stokes(image);
    
//     // polar componnets
//     //std::vector<cv::Mat> polar_images = 
//     //POLPro::compute_polar_params(stokes_images); 

//     //showing the paramters

//     //POLPro::imshow(angle_image); 

//     //POLPro::imshow(stokes_images); 

//     //POLPro::imshow(polar_images, false, false); 
    
//    waitKey(0); 
//    return 0 ; 
   
// }
