// this code uses optical flow to estimate camera velocity
//

#include "opencv2/opencv.hpp"
#include <iostream>
#include <ctime>
#include <cmath>
#define USE_MATH_DEFINES

using namespace std;
using namespace cv;


int main()
{
    // initialize variables
    int waitTime = 0; // milliseconds
    Mat framePrev;
    vector<Point2f> corners, cornersLK, cornersPrev;

    // parameters: goodFeaturesToTrack
    int maxCorners = 70; // keep best # features
    double qualityLevel = 0.01;
    double minDist = 5;
    int blockSize = 3;

    // parameters: calcOpticalFlowPyrLK
    vector<uchar> status;
    vector<float> err;
    Size winSize = Size(31,31);
    int maxLevel = 3;
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    // parameters: line and circle
    int radius = 2;
    int thickness = 1;
    Scalar lineColor = Scalar(0, 0, 255);
    Scalar circColor = Scalar(0, 255, 0);

    // parameters: findHomography
    int method = CV_RANSAC;
    double ransacReprojThreshold = 3;
    const int maxIters = 2000;
    const double confidence = 0.995;

    // create video capture object
    VideoCapture cap("../testvid.mp4");
    if (!cap.isOpened()) 
    {
    	cout << "Unable to read video file." << endl; 
    	return -1; 
    }

    // start timer
    clock_t t0 = clock();
    double tprev = 0;

    // loop through video frames
    for (;;)
    {
    	Mat frame, frameGray;
    	cap >> frame; // get new frame
    	if (frame.empty())
    	{
    		break;
    	}
    	cvtColor(frame, frameGray, COLOR_BGR2GRAY);

        // get time and time step
        double t = (double)(clock() - t0)/CLOCKS_PER_SEC;
        double ts = t - tprev;
        cout << ts << endl;

        // make up values of angular velocity
        double omega_x = 0.1*sin(t);
        double omega_y = 0.1*cos(t);
        double omega_z = 0.1*tan(t);
        Mat omega = (Mat_<double>(3,1) << omega_x, omega_y, omega_z);
        Mat omega_hat = (Mat_<double>(3,3) << 0, -omega_z, omega_y, omega_z, 0, -omega_x, -omega_y, omega_x, 0);

    	// find good features
    	Mat mask; // just to run function but not actually needed
        goodFeaturesToTrack(frameGray, corners, maxCorners, qualityLevel, minDist, mask, blockSize, false, 0.04);

        // compute optical flow after 2 sets of data is stored
        if (!framePrev.empty()) 
        {
        	Mat framePrevGray;
        	cvtColor(framePrev, framePrevGray, COLOR_BGR2GRAY);

            cornersLK = corners; // use these for LK to not lose original corners
            calcOpticalFlowPyrLK( framePrevGray, frameGray, cornersPrev, cornersLK, status, err, winSize, maxLevel, criteria, 0, 1e-4 );

            // get matching features
            vector<Point2f> cornersPrevGood, cornersGood;
            for (int i = 0; i < status.size(); i++) 
            {
                if ( status[i] ) {
                    cornersPrevGood.push_back(cornersPrev[i]);
                    cornersGood.push_back(cornersLK[i]);
                }
            }

            // compute homography matrix
            Mat mask; // just to run function but not actually needed
            Mat H = findHomography(cornersPrevGood, cornersGood, method, ransacReprojThreshold, mask, maxIters, confidence);

            // remove angular velocity from homography
            Mat H_no_omega = H - omega_hat;

            // decompose H_no_omega into velocity and normal components
            Mat Sigma, U, V, Vt;
            SVD::compute(H_no_omega, Sigma, U, Vt);
            transpose(Vt, V);
            Mat N = (Mat_<double>(3,1) << V.at<double>(0,0), V.at<double>(1,0), V.at<double>(2,0))*(-1);
            Mat v_over_d = H_no_omega*N;

            // draw matching features
            for (int i = 0; i < status.size(); i++) 
            {
                circle(framePrev, cornersPrevGood[i], radius, circColor, thickness, LINE_AA);
                line(framePrev, cornersPrevGood[i], cornersGood[i], lineColor, thickness, LINE_AA);
            }

            // show image
            imshow("framePrev", framePrev);
            char input_key = waitKey(waitTime);
            if (input_key == 'c')
    		{
    			break;
    		}
        }

        // store previous frame and corners
        framePrev = frame.clone();
        cornersPrev = corners;
        tprev = t; // store to to get time step next iterations (needed to ge pixel velocity)
    }

    return 0;
}
