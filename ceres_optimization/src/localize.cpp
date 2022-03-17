#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iostream>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <string.h>
#define RADIUS 20.0f

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr merged;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> global_octree(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f));

//[future work] remove global wariables
Eigen::Matrix<float, 3,4> D_Camera_Proj_fn;
Eigen::Matrix<float, 3,4> Cam_Proj;

//subscriber class to subscribe TCM(from VINS), depth,scharr image(opencv::mat), point cloud(pcl)

//[future work] add this class inside some function 
class Localize : public ceres::SizedCostFunction<1, 6>
{
public:
    Localize(Eigen::Matrix4f &TCM, Eigen::Vector4f p, cv::Mat &disparity, cv::Mat &scharredX, cv::Mat &scharredY ,Eigen::Matrix<float, 3,4> &D_Camera_Proj_fn, Eigen::Matrix<float, 3,4> &Cam_Proj)
    {
        Eigen::Matrix4f TCM_;
        Eigen::Vector4f p_;
        cv::Mat disparity_;
        cv::Mat scharredX_;
        cv::Mat scharredY_;
        Eigen::Matrix<float, 3,4> D_Camera_Proj_fn_;
        Eigen::Matrix<float, 3,4> Cam_Proj_;
    }
    virtual ~Localize() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {   
        Eigen::Matrix4f Teps_;
        Eigen::Matrix<float, 1, 3> depthJac;

        depthJac << scharredX_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), scharredY_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 1.0;
        Teps_ << 1.0, -parameters[0][2], parameters[0][1], parameters[0][3],
            parameters[0][2], 1.0, -parameters[0][0], parameters[0][4],
            -parameters[0][1], parameters[0][0], 1.0, parameters[0][5],
            0.0, 0.0, 0.0, 1.0;

        double sigma = sqrt(pow(scharredX_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 2) + pow(scharredY_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 2));
        residuals[0] = (((Teps_ * (TCM_ * p_))(2,0)) - disparity_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0))) / sigma;
        if (!jacobians)
            return true;
        double *jacobian = jacobians[0];
        if (!jacobian)
            return true;

        double x = (Teps_ * (TCM_ * p_))(0, 0);
        double y = (Teps_ * (TCM_ * p_))(0, 1);
        double z = (Teps_ * (TCM_ * p_))(0, 2);

        Eigen::Matrix<float, 3,4> D_Camera_Proj_fn = D_Camera_Proj_fn_;
        D_Camera_Proj_fn(0, 0) /= z;
        D_Camera_Proj_fn(1, 1) /= z;
        D_Camera_Proj_fn(0, 2) /= (z * z);
        D_Camera_Proj_fn(1, 2) /= (z * z);

        Eigen::Matrix4f temp = Eigen::Matrix4f::Zero();
        temp(1, 2) = -1.0;
        temp(2, 1) = 1.0;
        jacobian[0] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

        temp = Eigen::Matrix4f::Zero();
        temp(0, 2) = 1.0;
        temp(2, 0) = 1.0;
        jacobian[1] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

        temp = Eigen::Matrix4f::Zero();
        temp(0, 1) = -1.0;
        temp(1, 0) = 1.0;
        jacobian[2] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

        temp = Eigen::Matrix4f::Zero();
        temp(0, 3) = 1.0;
        jacobian[3] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

        temp = Eigen::Matrix4f::Zero();
        temp(1, 3) = 1.0;
        jacobian[4] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

        temp = Eigen::Matrix4f::Zero();
        temp(2, 3) = 1.0;
        jacobian[5] = (temp * (TCM_ * p_))(2,0) - ((depthJac) * (D_Camera_Proj_fn) * (temp * (TCM_ * p_)))(0,0);

    return true;
    }

private:
    Eigen::Matrix4f TCM_;
    Eigen::Vector4f p_;
    cv::Mat disparity_;
    cv::Mat scharredX_;
    cv::Mat scharredY_;
    Eigen::Matrix<float, 3,4> D_Camera_Proj_fn_;
    Eigen::Matrix<float, 3,4> Cam_Proj_;
};

void optimize(const nav_msgs::Odometry::ConstPtr &tf_msg, const sensor_msgs::Image::ConstPtr &depth_img){
    ROS_INFO("Call back function is being called.\n");
    // qt = tf_msg->transform.rotation;
    // t = tf_msg->transform.translation;

    google::InitGoogleLogging("file_name");
    ceres::Problem problem;
    double params[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::Vector4f t, p;
    p(3) = 1;
    t(0) = tf_msg->pose.pose.position.x;
    t(1) = tf_msg->pose.pose.position.y;
    t(2) = tf_msg->pose.pose.position.z;
    t(3) = 1;

    Eigen::Quaternionf q;
    q.x() = tf_msg->pose.pose.orientation.x;
    q.y() = tf_msg->pose.pose.orientation.y;
    q.z() = tf_msg->pose.pose.orientation.z;
    q.w() = tf_msg->pose.pose.orientation.w;

    Eigen::Matrix3f R = q.normalized().toRotationMatrix();
    Eigen::Matrix4f _TCM = Eigen::Matrix4f::Zero();
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            _TCM(i,j) = R(i,j);
        }
        _TCM(i,3) = t(i);
    }

    // _TCM.block<3, 3>(0, 0) = R;
    // _TCM.block<4, 1>(3, 0) = t; // Review

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(depth_img);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
    Mat scharrX, scharrY;
    cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);


    // cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
    // cv_ptr1 = cv_bridge::toCvCopy(depth_img);
    // cv::Mat depth_float_img = cv_ptr1->image;
    // cv::Mat depth_mono8_img;
    // if(depth_mono8_img.rows != depth_float_img.rows || depth_mono8_img.cols != depth_float_img.cols){
    // depth_mono8_img = cv::Mat(depth_float_img.size(), CV_8UC1);}
    // cv::convertScaleAbs(depth_float_img, depth_mono8_img, 100, 0.0);

    // cv_ptr1 = cv_bridge::toCvCopy(depth_img, std::string());
    // cv_ptr2 = cv_bridge::toCvCopy(scharr_img, sensor_msgs::image_encodings::BGR8);
    Scharr(mono8_img, scharrX, -1, 1, 0);
    Scharr(mono8_img, scharrY, -1, 0, 1);
    convertScaleAbs(scharrX, scharrX);
    convertScaleAbs(scharrY, scharrY);
    // int ddepth = CV_16S;
    // int ksize = 3;
    // Sobel(mono8_img, scharr, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);

    pcl::PointXYZ center(t(0), t(1), t(2));
    float radius = RADIUS;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ROS_INFO("Imaged read and now going into ceres solver.\n");
    if (global_octree.radiusSearch(center, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            p(0) = (*merged)[pointIdxRadiusSearch[i]].x;
            p(1) = (*merged)[pointIdxRadiusSearch[i]].y;
            p(2) = (*merged)[pointIdxRadiusSearch[i]].z;
            ROS_INFO("%ld points added to ceres solver\n", i);
            if ((Cam_Proj * (_TCM * p))(0) >= 0 && (Cam_Proj * (_TCM * p))(1) >= 0 && (Cam_Proj * (_TCM * p))(0) <= mono8_img.rows && (Cam_Proj * (_TCM * p))(1) <= mono8_img.cols)
            {
                ceres::CostFunction *cost_func = new Localize(_TCM, p, mono8_img, scharrX, scharrY, D_Camera_Proj_fn, Cam_Proj);
                problem.AddResidualBlock(
                    cost_func,
                    nullptr,
                    params);
            }
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ROS_INFO("Data loaded to the solver and solving is started\n");
        ceres::Solve(options, &problem, &summary);
        ROS_INFO("Solved gg.\n");
    }
    Eigen::Matrix4f teps = Eigen::Matrix4f::Identity();
    teps(0, 1) = -params[2];
    teps(1, 0) = params[2];
    teps(0, 2) = params[1];
    teps(2, 0) = -params[1];
    teps(1, 2) = -params[0];
    teps(2, 1) = params[0];
    teps(0, 3) = params[3];
    teps(1, 3) = params[4];
    teps(2, 3) = params[5];

    _TCM = teps * _TCM;
    cout << _TCM << endl;
}

//[future work]add functions for subscribing dpeth image and vins output
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localize_node");
    ros::NodeHandle nh_localize_node;
    double fx, fy, s, cx, cy;

    ros::NodeHandle nh;
    ros::Publisher pub;

    std::string depth_topic, scharr_topic, vins_topic, pcd_path;

    bool param_success = true;
    string key;
    string tempKey;

    if (!ros::param::search("depth_topic_name", key))
        param_success = false;
    ros::param::get(key, depth_topic);
    if (!ros::param::search("scharr_topic_name", key))
        param_success = false;
    ros::param::get(key, scharr_topic);
    if (!ros::param::search("vins_topic_name", key))
        param_success = false;
    ros::param::get(key, vins_topic);
    if (!ros::param::search("pcd_path", key))
        param_success = false;
    ros::param::get(key, pcd_path);

    if (!ros::param::search("fx", key))
        param_success = false;
    ros::param::get(key, tempKey);
    fx = atof(tempKey.c_str());

    if (!ros::param::search("fy", key))
        param_success = false;
    ros::param::get(key, tempKey);
    fy = atof(tempKey.c_str());

    if (!ros::param::search("cx", key))
        param_success = false;
    ros::param::get(key, tempKey);
    cx = atof(tempKey.c_str());

    if (!ros::param::search("cy", key))
        param_success = false;
    ros::param::get(key, tempKey);
    cy = atof(tempKey.c_str());

    // if (!ros::param::search("s", key))
    //     param_success = false;
    // ros::param::get(key, s);

    D_Camera_Proj_fn << fx, 0.0, -fx, 0.0,
        0.0, fy, -fy, 0.0,
        0.0, 0.0, 0.0, 1.0;
    Cam_Proj << fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *(merged)) == -1) // load the file
    {
        ROS_INFO("Couldn't read file\n");
        param_success = false;
    }

    global_octree.setInputCloud(merged);
    global_octree.addPointsFromInputCloud();


    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> scharr_sub(nh, scharr_topic, 10);

    message_filters::Subscriber<nav_msgs::Odometry> vins_sub(nh, vins_topic, 10);

    // Ref: https://stackoverflow.com/questions/62267850/ros-c-approximate-time-synchronizer-callback-not-working-in-node-member-class
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vins_sub, depth_sub);

    ROS_INFO("Subscribed topics now going into callback.\n");
    sync.registerCallback(boost::bind(&optimize, this, _1, _2));
    ros::spin();

            
    return 0;
}