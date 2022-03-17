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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "static_matcher");
    ros::NodeHandle nh_static;

    int numDisparities;
    int blockSize;
    int disp12MaxDiff;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int mindisaprity;
    int prefilterCap;
    string name1;
    string name2;

    Mat img1, img2, imgtruth, disp, display,depth;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create();
    namedWindow("left", WINDOW_NORMAL);
    namedWindow("right", WINDOW_NORMAL);
    namedWindow("dispshow", WINDOW_NORMAL);
    //namedWindow("truthshow", WINDOW_NORMAL);
    //namedWindow("errorshow", WINDOW_NORMAL);

    cout << "Images Loaded\n";
    int number = 0;
    long error = 0;
    ros::Rate loop_rate(300);
    while (ros::ok()&&number<200)
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
        // param_success &= nh_static.getParam("name1", name1);
        // param_success &= nh_static.getParam("name2", name2);

        if (!param_success)
        {
            ROS_INFO("Parameters not loaded Loaded.");
        }
        ///home/pranav/Downloads/data_scene_flow/testing/image_2/000000_10.png
        string seq = "000000";
        int temp = number;
        for (int i = 5; i >= 0; i--)
        {
            seq[i] += temp % 10;
            temp = temp / 10;
        }
        string tempnew = "000000";
        img1 = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/" + seq + "_10.png");
        img2 = imread("/home/aditya/catkin_ws/src/disparity_map/src/InputImages/" + seq + "_11.png");
        // equalizeHist(img1,img1);
        // equalizeHist(img2,img2);
        //cout << "Images Loaded." << name1 << endl;

        sgbm->setMinDisparity(mindisaprity);
        sgbm->setPreFilterCap(prefilterCap);
        sgbm->setP1(4 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setP2(32 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setBlockSize((2 * blockSize + 1));
        sgbm->setNumDisparities(16 * numDisparities);
        sgbm->setUniquenessRatio(uniquenessRatio);
        sgbm->setSpeckleWindowSize(speckleWindowSize);
        sgbm->setSpeckleRange(speckleRange);
        sgbm->setDisp12MaxDiff(disp12MaxDiff);
        sgbm->setMode(StereoSGBM::MODE_HH);
        sgbm->compute(img1, img2, disp);
        normalize(disp,disp,0,255,CV_MINMAX,CV_8UC1);
        //normalize(disp, disp, 0, 255, CV_MINMAX, CV_8UC3);
        //imgtruth = imread("/home/pranav/Downloads/data_scene_flow/training/disp_noc_0/" + seq + "_10.png", 0);
        //display = imread("/home/pranav/Downloads/data_scene_flow/training/disp_noc_0/" + seq + "_10.png", 0);
        //normalize(imgtruth, imgtruth, 0, 255, CV_MINMAX, CV_8U);
        //absdiff(disp, imgtruth, display);
        //cout << "Absolute Diff Working\n";
        imshow("left", img1);
        imshow("right", img2);
        imshow("dispshow", disp);
        //imshow("truthshow", imgtruth);
        //imshow("errorshow", display);
        //cout << norm(display, NORM_L2, noArray()) << endl;
        //imwrite("/home/pranav/Downloads/sample2/disparity" + std::to_string(number) + ".png", disp);
        //error += norm(display, NORM_L2, noArray()) / 200;
        imwrite("/home/aditya/catkin_ws/src/disparity_map/src/output/"+std::to_string(number)+".png",disp);
        number++;
        waitKey(300);
    }
    //cout << error << endl;

    loop_rate.sleep();

    destroyAllWindows();
    return 0;
}