#include "ceres/ceres.h"
#include "glog/logging.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <string.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/impl/point_types.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
using namespace cv;

class Localize : public ceres::SizedCostFunction<1, 6>
{
public:
    Localize(Eigen::Matrix4f &TCM, Eigen::Vector4f p, cv::Mat &disparity, cv::Mat &scharredX, cv::Mat &scharredY ,Eigen::Matrix<float, 3,4> &D_Camera_Proj_fn, Eigen::Matrix<float, 3,4> &Cam_Proj);
    virtual ~Localize();
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;

    Eigen::Matrix4f TCM_;
    Eigen::Vector4f p_;
    cv::Mat disparity_;
    cv::Mat scharredX_;
    cv::Mat scharredY_;
    Eigen::Matrix<float, 3,4> D_Camera_Proj_fn_;
    Eigen::Matrix<float, 3,4> Cam_Proj_;
};

class Handler
{
public:
    Handler(ros::NodeHandle &nodeHandle);
    virtual ~Handler();
    bool readParams();
    void optimize(const nav_msgs::Odometry::ConstPtr &tf_msg,
                  const sensor_msgs::Image::ConstPtr &depth_img
                  );
    void run();

private:
    Eigen::Matrix<float, 3,4> D_Camera_Proj_fn;
    Eigen::Matrix<float, 3,4> Cam_Proj;
    double fx, fy, s, cx, cy;

    ros::NodeHandle nh;
    ros::Publisher pub;

    std::string depth_topic, scharr_topic, vins_topic, pcd_path;

    PointCloud::Ptr merged;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> global_octree;
    int count;
    //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> global_octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::OctreePointCloudSearch(128.0f);
};