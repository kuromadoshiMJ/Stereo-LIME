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
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle nh_static;
    namedWindow("depth", WINDOW_NORMAL);
    int number=0;Mat disp;Mat depth;
    ros::Rate loop_rate(30);
    while(ros::ok()&&number<200)
    {
        int temp = number;
        string s=to_string(temp);
        disp = imread("/home/aditya/catkin_ws/src/disparity_map/src/output2/" + s + ".png");
        // disp.convertTo(disp,CV_32FC1);
        // disp=disp/256;
        depth=(721.5377*0.54)/disp;
        normalize(depth,depth,0,255,CV_MINMAX,CV_8UC1);
        imshow("depth", depth);
        imwrite("/home/aditya/catkin_ws/src/disparity_map/src/depth_output2/"+std::to_string(number)+".png",depth);
        number++;
        waitKey(30);
    }
    loop_rate.sleep();

    destroyAllWindows();
    return 0;
}