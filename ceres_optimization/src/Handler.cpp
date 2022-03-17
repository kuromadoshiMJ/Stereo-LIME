#include <ceres_optimization.hpp>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
using namespace cv;
#define RADIUS 20.0f

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 1, 0.0);
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
}

Handler::Handler(ros::NodeHandle &nodeHandle) : nh(nodeHandle), global_octree(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f))
{
    pub = nh.advertise<geometry_msgs::TransformStamped>("stereo_output", 10);
    global_octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f);
    merged = PointCloud::Ptr(new PointCloud);
    count = 0;
    if (!readParams())
    {
        ROS_INFO("Parameters were not read.");
        ros::requestShutdown();
    }
    else
    {
        ROS_INFO("Parameters readed .");
    }
}

Handler::~Handler()
{
}

bool Handler::readParams()
{
    string key;
    string tempKey;

    if (!ros::param::search("depth_topic_name", key))
        return false;
    ros::param::get(key, Handler::depth_topic);
    if (!ros::param::search("scharr_topic_name", key))
        return false;
    ros::param::get(key, Handler::scharr_topic);
    if (!ros::param::search("vins_topic_name", key))
        return false;
    ros::param::get(key, Handler::vins_topic);
    if (!ros::param::search("pcd_path", key))
        return false;
    ros::param::get(key, Handler::pcd_path);

    if (!ros::param::search("fx", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::fx = atof(tempKey.c_str());

    if (!ros::param::search("fy", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::fy = atof(tempKey.c_str());

    if (!ros::param::search("cx", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::cx = atof(tempKey.c_str());

    if (!ros::param::search("cy", key))
        return false;
    ros::param::get(key, tempKey);
    Handler::cy = atof(tempKey.c_str());

    // if (!ros::param::search("s", key))
    //     return false;
    // ros::param::get(key, Handler::s);

    D_Camera_Proj_fn << fx, 0.0, -fx, 0.0,
        0.0, fy, -fy, 0.0,
        0.0, 0.0, 0.0, 1.0;
    Cam_Proj << fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(Handler::pcd_path, *(Handler::merged)) == -1) // load the file
    {
        ROS_INFO("Couldn't read file\n");
        return false;
    }

    Handler::global_octree.setInputCloud(Handler::merged);
    Handler::global_octree.addPointsFromInputCloud();
    return true;
}

void Handler::run()
{
    cout << depth_topic << endl;
    cout << vins_topic << endl;

    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> scharr_sub(nh, scharr_topic, 10);

    message_filters::Subscriber<nav_msgs::Odometry> vins_sub(nh, vins_topic, 10);

    // Ref: https://stackoverflow.com/questions/62267850/ros-c-approximate-time-synchronizer-callback-not-working-in-node-member-class
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vins_sub, depth_sub);

    ROS_INFO("Subscribed topics now going into callback.\n");
    sync.registerCallback(boost::bind(&Handler::optimize, this, _1, _2));
    ros::spin();
}

void Handler::optimize(const nav_msgs::Odometry::ConstPtr &tf_msg,
                       const sensor_msgs::Image::ConstPtr &depth_img)
{   
    count++;
    ROS_INFO("Call back function is being called.\n");
    // qt = tf_msg->transform.rotation;
    // t = tf_msg->transform.translation;

    // google::InitGoogleLogging("file_name");
    ceres::Problem problem;
    double params[] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

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

    Eigen::Matrix3f R = q.toRotationMatrix(); // q.normalized().toRotationMatrix();
    // ROS_INFO_STREAM("----------------------------------------\n"<<R.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    Eigen::Matrix4f _TCM = Eigen::Matrix4f::Zero();
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            _TCM(i,j) = R(i,j);
        }
        _TCM(i,3) = t(i);
    }
    _TCM(3,3) = 1;
    // ROS_INFO_STREAM("----------------------------------------\n"<<t.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    // ROS_INFO_STREAM("----------------------------------------\n"<<_TCM.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    // _TCM.block<3, 3>(0, 0) = R;
    // _TCM.block<4, 1>(3, 0) = t; // Review

     cv_bridge::CvImagePtr cv_ptr;
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(depth_img);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat mono8_img;
    cv::Mat scharrX, scharrY;
    depthToCV8UC1(depth_float_img, mono8_img);

    // cv_bridge::CvImageConstPtr cv_ptr;
    // try
    // {
    //     cv_ptr = cv_bridge::toCvShare(depth_img);

    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
    // Mat scharrX, scharrY;
    // cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);

    // cv_bridge::CvImagePtr cv_ptr1;
    // cv_ptr1 = cv_bridge::toCvCopy(depth_img,sensor_msgs::image_encodings::BGR8);


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
    string name1, name2, name3, name4;
    // name1 = "/home/pranav/ws/src/depth/depth1_";
    // name1+=std::to_string(count);
    // name1+=".png";
    // imwrite(name1, mono8_img);
    // name2 = "/home/pranav/ws/src/scharr/scharrX_";
    // name2+=std::to_string(count);
    // name2+=".png";
    // imwrite(name2,scharrX);
    // name3 = "/home/pranav/ws/src/scharr/scharrY_";
    // name3+= std::to_string(count);
    // name3+= ".png";
    // imwrite(name3,scharrY);
    // name1 = "/home/pranav/ws/src/depth/depth2_";
    // name1+=std::to_string(count);
    // name1+=".png";
    // imwrite(name4, cv_ptr->image);
    // int ddepth = CV_16S;
    // int ksize = 3;
    // Sobel(mono8_img, scharr, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);

    pcl::PointXYZ center(t(0), t(1), t(2));
    float radius = RADIUS;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ROS_INFO("Images read and now going into ceres solver.\n");
    if (global_octree.radiusSearch(center, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            p(0) = (*merged)[pointIdxRadiusSearch[i]].x - t(0);
            p(1) = (*merged)[pointIdxRadiusSearch[i]].y - t(1);
            p(2) = (*merged)[pointIdxRadiusSearch[i]].z - t(2);
            // ROS_INFO("%ld points added to ceres solver\n", i);
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
        // ROS_INFO("Data loaded to the solver and solving is started\n");
        // ceres::Solve(options, &problem, &summary);
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

    ROS_INFO_STREAM("\n"<<_TCM.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]")));
    _TCM = teps * _TCM;
    ROS_INFO_STREAM("\n\n"<<_TCM.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    cout << _TCM << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ceres_optimiser");
    ros::NodeHandle nh_ceres;
    
    google::InitGoogleLogging("file_name");
    Handler a(nh_ceres);
    a.run();

    return 0;
}