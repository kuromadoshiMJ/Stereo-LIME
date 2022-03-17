#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdio.h>
#include <string.h>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
//#include "opencv2/contrib/contrib.hpp"
using namespace message_filters;
using namespace cv;
using namespace std;

int counter=0;

void Callback(const sensor_msgs::Image::ConstPtr &msg1, const sensor_msgs::Image::ConstPtr &msg2)
{   if(counter%100==0){
        cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    Mat img1, img2;

    cv_ptr1 = cv_bridge::toCvCopy(msg1,sensor_msgs::image_encodings::BGR8);
    cv_ptr2 = cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::BGR8);

    img1 = cv_ptr1->image;
    img2 = cv_ptr2->image;
    

    string name1;
    string name2;

    name1 = "~/Downloads/sample_stereo_imgs/img1_" ;
    name1 += std::to_string(counter);
    name1 +=  ".png" ;
    name2 = "~/Downloads/sample_stereo_imgs/img2_" ;
    name2 += std::to_string(counter);
    name2 +=  ".png" ;

    cout<<name1<<endl;
    cout<<name2<<endl;

    imwrite(name1,img1);
    imwrite(name2,img1);
}
    counter++;
    

    return;
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imgwriter");
    ros::NodeHandle nh_imgwriter;
    message_filters::Subscriber<sensor_msgs::Image> input1(nh_imgwriter, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> input2(nh_imgwriter, "/cam1/image_raw", 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(input1, input2, 1);
    ROS_INFO("Going into callback..");
    sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();
    
    return 0;
}