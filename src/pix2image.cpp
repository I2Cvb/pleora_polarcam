#include <pix2image.h>
#include <photonfocus_camera.h>


namespace POLPro
{
    Pix2Image::Pix2Image(cv::Mat img)
    {
	cv_bridge::CvImage cv_image; 
	cv_image.encoding = "mono8"; 
	cv_image.image = img; 
	
	
    }
    
    void Pix2Image::pix2parsed (cv::Mat img)
    {

	using namespace cv;
	using namespace std;

	Mat output = img.clone();
	cout << img.type() << "   " << CV_8U << std::endl;
	int cols = img.cols/2;
	int rows = img.rows/2;

	for (int i=0; i<rows;i++){
	    for (int j=0; j<cols;j++){
		output.at<uchar>(i, j) = img.at<uchar>(2*i, 2*j);
		output.at<uchar>(i+rows-1, j) = img.at<uchar>(2*i+1, 2*j);
		output.at<uchar>(i, j+cols-1) = img.at<uchar>(2*i, 2*j+1);
		output.at<uchar>(i+rows-1, j+cols-1) = img.at<uchar>(2*i+1, 2*j+1);
		}
	    }
	imshow( "Parsed image", output ); // Show our image inside it. 

	waitKey(0);                   // Wait for a keystroke in the window 
	return output; 
	    
    }

    void Pix2Image::pix2rgb (cv::Mat img)
    {

	double min, max;
	Point idmin, idmax;
	minMaxLoc(img, &min, &max, &idmin, &idmax) ;
	cout <<"min max src: " << min << " " << max << std::endl;

	int cols = img.cols/2;
	int rows = img.rows/2;
	Mat I0 = Mat(rows, cols, img.type());
	Mat I45 = Mat(rows, cols, img.type());
	Mat I90 = Mat(rows, cols, img.type());
	Mat I135 = Mat(rows, cols, img.type());

	for (int i=0; i<rows;i++){
	    for (int j=0; j<cols;j++){
		I0.at<uchar>(i, j) = img.at<uchar>(2*i, 2*j);
		I45.at<uchar>(i, j) = img.at<uchar>(2*i+1, 2*j);
		I90.at<uchar>(i, j) = img.at<uchar>(2*i, 2*j+1);
		I135.at<uchar>(i, j) = img.at<uchar>(2*i+1, 2*j+1);
	    }
	}
       // S0 
	Mat s0;
	add(I0, I90, s0, noArray(), CV_32F);
	add(I45, s0, s0, noArray(), CV_32F);
	add(I135, s0, s0, noArray(), CV_32F);
	s0 = s0 / 2.0;
	minMaxLoc(s0, &min, &max, &idmin, &idmax) ;
	cout <<"min max s0: " << min << " " << max << std::endl;


	//S1  
        Mat s1;
        subtract(I0, I90, s1, noArray(), CV_32F);
        minMaxLoc(s1, &min, &max, &idmin, &idmax) ;
        cout <<"min max s1: " << min << " " << max << std::endl;
        //S2                                                    
        Mat s2;
        subtract(I45, I135, s2, noArray(), CV_32F);
        minMaxLoc(s2, &min, &max, &idmin, &idmax) ;
        cout <<"min max s2: " << min << " " << max << std::endl;

        // DOP and AOP parameters                               
	cv::Mat dop, aop;
	cv::cartToPolar(s1, s2, dop, aop, true); // Provide angle in degree   
	dop = dop / s0;
	aop = 0.5 * aop;


        /* Conversion into 8 bits depth image */
        s0 = s0/2;
        s1 = (s1+255.0)/2;
        s2 = (s2+255.0)/2;
        s0.convertTo(s0, CV_8UC1);
        s1.convertTo(s1, CV_8UC1);
        s2.convertTo(s2, CV_8UC1);

	/* HSV representation */
	cv::Mat S = dop * 255; // S in the range 0:255
	cv::Mat H = aop; // H in the range 0:180      
	S.convertTo(S, CV_8UC1);
	H.convertTo(H, CV_8UC1);

	vector<cv::Mat> channels;
	channels.push_back(H);
	channels.push_back(S);
	channels.push_back(s0);

	Mat img_hsv;
	merge(channels, img_hsv);

	Mat HSL;
	cvtColor(img_hsv, HSL, CV_HLS2BGR);

	Mat Stokes = Mat(rows*2, cols*2, image.type());
	s0.copyTo(Stokes(cv::Rect(0, 0, s0.cols, s0.rows)));
	s1.copyTo(Stokes(cv::Rect(cols, 0, s1.cols, s1.rows)));
	s2.copyTo(Stokes(cv::Rect(0, rows, s2.cols, s2.rows)));

	Mat PolFea = Mat(rows*2, cols*2, image.type());
	S.copyTo(PolFea(cv::Rect(0, 0, S.cols, S.rows))); //DoP 
	H.copyTo(PolFea(cv::Rect(cols, 0, H.cols, H.rows))); //AoP
	/* show images */
	imshow( "Pixelated image", image);
	imshow("hsv", img_hsv);
	imshow("Stokes", Stokes);
	imshow("Polfea", PolFea);
	waitKey(0);                                          // Wait for a keystroke in the window

	return 0;
    }



}


