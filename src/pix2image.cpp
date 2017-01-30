#include <pix2image.h>
#include <photonfocus_camera.h>


namespace POLPro
{

    cv::Mat raw2mat(const cv::Mat& origin)
    {
        // create the output image with the same dimension than the original
        // image. The raw image is a 8 bits image.
        cv::Mat output(origin.height(), origin.width(), cv::CV_8U);

        // define the size of the image without the useful info
        int cols = origin.width() / 2;
        int rows = origin.height() / 2;

        // re-order the data keeping only the useful information
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                // get 0 degree
                output.at<uchar>(i, j) = origin.at<uchar>(2 * i, 2 * j);
                // get 45 degrees
                output.at<uchar>(i + rows - 1, j) = origin.at<uchar>(2 * i + 1,
                                                                  2 * j);
                // get 90 degrees
                output.at<uchar>(i, j + cols - 1) = origin.at<uchar>(2 * i,
                                                                  2 * j + 1);
                // get 135 degrees
                output.at<uchar>(i + rows - 1, j + cols - 1) = origin.at<uchar>(
                    2 * i + 1, 2 * j + 1);
            }
        }
        // return the image
        return output;
    }

    std::vector<cv::Mat> raw2mat4d(const cv::Mat& origin)
    {
        // we will loop other the data
        std::vector<cv::Mat> output;

        // compute the size of the output image
        int cols = img.cols / 2;
        int rows = img.rows / 2;
        // Create the four angles images
        for (int i = 0; i < 4; ++i)
        {
            output.push_back(cv::Mat::zeros(rows, cols, cv::CV_8U))
        }

        // Copy the data
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                // Get the different angles
                for (int angles = 0; angles < 4; ++angles)
                {
                    int r = angles / 2;
                    int c = angles % 2;
                    output[angles].at<uchar>(i, j) = origin.at<uchar>(
                        2 * i + r, 2 * j + c);
                }

        // Return the image
        return output;
    }

    cv::Mat compute_stokes(const cv::Mat& origin)
    {
        // compute the min and max of the image
        int min = 0, max = 0;
        cv::Point id_min, id_max;
        minMaxLoc(origin, &min, &max, &id_min, &id_max);
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
