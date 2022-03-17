#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdio.h>
#include <string.h>

using namespace cv;
using namespace std;
using namespace cv::ximgproc;
int main(int argc,char **argv)
{
    //ROS node initialisation
    ros::init(argc,argv,"filtered_static_matcher");
    ros::NodeHandle nh_static;

    //Parameters initialization
    int numDisparities;
    int blockSize;
    int disp12MaxDiff;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int mindisaprity;
    int prefilterCap;
    double lamda;
    double sigma;

    Mat left,right,left_for_matcher,right_for_matcher,left_disp,right_disp,filtered_disp;
    Ptr<DisparityWLSFilter> wls_filter;
    namedWindow("left", WINDOW_NORMAL);
    namedWindow("right", WINDOW_NORMAL);

    ros::Rate loop_rate(30);
    int number=0;
    while(ros::ok()&&number<200)
    {
        bool param_success = true;

        param_success &= nh_static.getParam("numDisparities", numDisparities);
        param_success &= nh_static.getParam("SADWindowSize", blockSize);
        param_success &= nh_static.getParam("uniquenessRatio", uniquenessRatio);
        param_success &= nh_static.getParam("speckleWindowSize", speckleWindowSize);
        param_success &= nh_static.getParam("speckleRange", speckleRange);
        param_success &= nh_static.getParam("disp12MaxDiff", disp12MaxDiff);
        param_success &= nh_static.getParam("mindisaprity", mindisaprity);
        param_success &= nh_static.getParam("prefilterCap", prefilterCap);
        param_success &= nh_static.getParam("wls_lamda", lamda);
        param_success &= nh_static.getParam("wls_sigma", sigma);
        if (!param_success)
        {
            ROS_INFO("Parameters not loaded Loaded.");
        }
        string seq = "000000";
        int temp = number;
        for (int i = 5; i >= 0; i--)
        {
            seq[i] += temp % 10;
            temp = temp / 10;
        }
        string tempnew = "000000";
        left = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/" + seq + "_10.png", 0);
        right = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/" + seq + "_11.png", 0);

        //Downsampling
        // int max_disp=16*numDisparities+mindisaprity;
        // max_disp/=2;
        // if(max_disp%16!=0)
        //     max_disp+=16-(max_disp%16);
        // resize(left,left_for_matcher,Size(),0.5,0.5,INTER_LINEAR_EXACT);
        // resize(right,right_for_matcher,Size(),0.5,0.5,INTER_LINEAR_EXACT);
        left_for_matcher=left.clone();
        right_for_matcher=right.clone();

        Ptr<StereoSGBM> left_matcher=StereoSGBM::create();
        left_matcher->setMinDisparity(mindisaprity);
        left_matcher->setNumDisparities(16*numDisparities);
        left_matcher->setBlockSize(2*blockSize+1);
        left_matcher->setP1(8 * (2 * blockSize + 1) * (2 * blockSize + 1));
        left_matcher->setP2(32 * (2 * blockSize + 1) * (2 * blockSize + 1));
        left_matcher->setPreFilterCap(prefilterCap);
        left_matcher->setDisp12MaxDiff(disp12MaxDiff);
        left_matcher->setPreFilterCap(prefilterCap);
        left_matcher->setSpeckleRange(speckleRange);
        left_matcher->setSpeckleWindowSize(speckleWindowSize);
        left_matcher->setUniquenessRatio(uniquenessRatio);
        left_matcher->setMode(StereoSGBM::MODE_HH);

        wls_filter=createDisparityWLSFilter(left_matcher);

        Ptr<StereoMatcher> right_matcher=createRightMatcher(left_matcher);

        left_matcher->compute(left_for_matcher,right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher,right_disp);

        wls_filter->setLambda(lamda);
        wls_filter->setSigmaColor(sigma);

        wls_filter->filter(left_disp,left,filtered_disp,right_disp);
        Mat conf_map=wls_filter->getConfidenceMap();
        Rect ROI=wls_filter->getROI();

        // resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
        // left_disp = left_disp*2.0;
        ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        
        Mat raw_disp_vis;
        imshow("left",left);
        imshow("right",right);
        getDisparityVis(left_disp,raw_disp_vis,1.00);
        namedWindow("raw disparity",WINDOW_AUTOSIZE);
        imshow("raw disparity",raw_disp_vis);
        Mat filtered_disp_vis;
        getDisparityVis(filtered_disp,filtered_disp_vis,1.00);
        namedWindow("wls disparity", WINDOW_AUTOSIZE);
        imshow("wls disparity", filtered_disp_vis);
        imwrite("/home/aditya/catkin_ws/src/disparity_map/src/output_WlsFilter/"+std::to_string(number)+".png",filtered_disp_vis);
        number++;
        waitKey(30);
    }
    loop_rate.sleep();

    destroyAllWindows();
    return 0;   
}