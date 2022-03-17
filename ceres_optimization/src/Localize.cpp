#include <ceres_optimization.hpp>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
using namespace cv;

// VINS,depth map extraction, output of lego loam bor on carla, bag file - reuired

Localize::Localize(Eigen::Matrix4f &TCM, Eigen::Vector4f p, Mat &disparity, Mat &scharredX, Mat &scharredY ,Eigen::Matrix<float, 3,4> &D_Camera_Proj_fn, Eigen::Matrix<float, 3,4> &Cam_Proj)
{
    TCM_ = TCM;
    p_ = p;
    disparity_ = disparity;
    scharredX_ = scharredX;
    scharredY_ = scharredY;
    D_Camera_Proj_fn_ = D_Camera_Proj_fn;
    Cam_Proj_ = Cam_Proj;
}

Localize::~Localize()
{
}

bool Localize::Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const
{
    // ROS_INFO("EVAL start");
    Eigen::Matrix4f Teps_;
    Eigen::Matrix<float, 1, 3> depthJac;
    // ROS_INFO("EVAL 0");
    Teps_ << 1.0, -parameters[0][2], parameters[0][1], parameters[0][3],
        parameters[0][2], 1.0, -parameters[0][0], parameters[0][4],
        -parameters[0][1], parameters[0][0], 1.0, parameters[0][5],
        0.0, 0.0, 0.0, 1.0;
    // ROS_INFO_STREAM("----------------------------------------\n"<<(Cam_Proj_ * (Teps_ * (TCM_ * p_))).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    // ROS_INFO_STREAM("----------------------------------------\n"<<((Teps_ * (TCM_ * p_))).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]"))<<"\n----------------------------------------\n");
    // ROS_INFO("Values: %f : %f", (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0));
    depthJac << scharredX_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), scharredY_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 1.0;
    
    // ROS_INFO("EVAL 1");
    double sigma = sqrt(pow(scharredX_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 2) + pow(scharredY_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0)), 2));
    residuals[0] = (((Teps_ * (TCM_ * p_))(2,0)) - disparity_.at<uchar>((Cam_Proj_ * (Teps_ * (TCM_ * p_)))(0,0), (Cam_Proj_ * (Teps_ * (TCM_ * p_)))(1,0))) / sigma;
    if (!jacobians)
        return true;
    double *jacobian = jacobians[0];
    if (!jacobian)
        return true;
    // ROS_INFO("EVAL 2");
    double x = (Teps_ * (TCM_ * p_))(0, 0);
    double y = (Teps_ * (TCM_ * p_))(1, 0);
    double z = (Teps_ * (TCM_ * p_))(2, 0);
    // ROS_INFO("EVAL 3");
    Eigen::Matrix<float, 3,4> D_Camera_Proj_fn = Localize::D_Camera_Proj_fn_; //.replicate<1,1>();
    D_Camera_Proj_fn(0, 0) /= z;
    D_Camera_Proj_fn(1, 1) /= z;
    D_Camera_Proj_fn(0, 2) /= (z * z);
    D_Camera_Proj_fn(1, 2) /= (z * z);
    // ROS_INFO("EVAL 4");
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
    // ROS_INFO("EVAL 4");
    return true;
}