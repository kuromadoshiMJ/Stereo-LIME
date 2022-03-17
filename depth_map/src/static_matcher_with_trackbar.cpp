#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/stereo.hpp>
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

int numDisparities=12;
int blockSize=2;
int disp12MaxDiff=10;
int uniquenessRatio=1;
int speckleWindowSize=150;
int speckleRange=2;
int mindisaprity=-64;
int prefilterCap=4;
string name1;
string name2;

Ptr<StereoSGBM> sgbm = StereoSGBM::create();

static void on_trackbar1(int, void*)
{
    sgbm->setNumDisparities(16*numDisparities);
    
}
static void on_trackbar2(int, void*)
{
    sgbm->setBlockSize((2 * blockSize + 1));
    
}
static void on_trackbar3(int, void*)
{
    sgbm->setDisp12MaxDiff(disp12MaxDiff);
}
static void on_trackbar4(int, void*)
{
    sgbm->setSpeckleWindowSize(speckleWindowSize);
}
static void on_trackbar5(int, void*)
{
    sgbm->setSpeckleRange(speckleRange);
}
static void on_trackbar6(int, void*)
{
    sgbm->setMinDisparity(mindisaprity);
}
static void on_trackbar7(int, void*)
{
    sgbm->setUniquenessRatio(uniquenessRatio);
}
static void on_trackbar8(int,void*)
{
    sgbm->setPreFilterCap(prefilterCap);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "static_matcher");
    ros::NodeHandle nh_static;

    Mat img1, img2, disp;
    
    namedWindow("left", WINDOW_NORMAL);
    namedWindow("right", WINDOW_NORMAL);
    namedWindow("dispshow", WINDOW_NORMAL);
    resizeWindow("dispshow",600,600);

    //cout << "Images Loaded\n";
    
    int number = 0;
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
         bool param_success = true;

        // param_success &= nh_static.getParam("numDisparities", numDisparities);
        // param_success &= nh_static.getParam("SADWindowSize", blockSize);
        // param_success &= nh_static.getParam("uniquenessRatio", uniquenessRatio);
        // param_success &= nh_static.getParam("speckleWindowSize", speckleWindowSize);
        // param_success &= nh_static.getParam("speckleRange", speckleRange);
        //param_success &= nh_static.getParam("disp12MaxDiff", disp12MaxDiff);
        param_success &= nh_static.getParam("mindisaprity", mindisaprity);
        // param_success &= nh_static.getParam("prefilterCap", prefilterCap);
        // param_success &= nh_static.getParam("name1", name1);
        // param_success &= nh_static.getParam("name2", name2);

        // if (param_success)
        // {
        //     ROS_INFO("Parameters Loaded.");
        // }
        ///home/pranav/Downloads/data_scene_flow/testing/image_2/000000_10.png

        createTrackbar("numDisparities","dispshow",&numDisparities,22,on_trackbar1);
        createTrackbar("blockSize","dispshow",&blockSize,25,on_trackbar2);
        createTrackbar("disp12MaxDiff","dispshow",&disp12MaxDiff,50,on_trackbar3);
        createTrackbar("speckleWindowSize","dispshow",&speckleWindowSize,200,on_trackbar4);
        createTrackbar("speckleRange","dispshow",&speckleRange,4,on_trackbar5);
        //createTrackbar("minDisparity","dispshow",&mindisaprity,-64,on_trackbar6);
        createTrackbar("UniquenessRatio","dispshow",&uniquenessRatio,15,on_trackbar7);
        createTrackbar("prefiltercap","dispshow",&prefilterCap,35,on_trackbar8);
        string seq = "000000";
        int temp = number;
        for (int i = 5; i >= 0; i--)
        {
            seq[i] += temp % 10;
            temp = temp / 10;
        }
        
        img1 = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/000004_10.png", 1);
        img2 = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/000004_11.png", 1);
        cout << "Images Loaded." << name1 << endl;
        
        
        sgbm->setMinDisparity(mindisaprity);
        sgbm->setP1(8 * (2*blockSize+1) * (2*blockSize+1));
        sgbm->setP2(32 * (2*blockSize+1) * (2*blockSize+1));
        sgbm->compute(img1, img2, disp);

        
        normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

        imshow("left", img1);
        imshow("right", img2);
        imshow("dispshow", disp);
        

        number++;
        waitKey(500);
    }

    loop_rate.sleep();

    destroyAllWindows();
    return 0;
}