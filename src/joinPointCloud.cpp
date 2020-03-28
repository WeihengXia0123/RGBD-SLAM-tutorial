#include <iostream>
using namespace std;

#include "slamBase.h"

// openCV库
#include <opencv2/core/eigen.hpp>

// eigen库
#include <Eigen/Core>
#include <Eigen/Geometry>

// pcl库
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    ParameterReader paraReader;
    // 声明两个帧,帧结构见slamBase.h
    FRAME frame1, frame2;

    // 读取图像
    frame1.rgb = cv::imread("../data/rgb1.png");
    frame1.depth = cv::imread("../data/depth1.png", -1); //flag=-1时，8位深度，原通道
    frame2.rgb = cv::imread("../data/rgb2.png");
    frame2.depth = cv::imread("../data/depth2.png", -1); //flag=-1时，8位深度，原通道

    // 提取特征关键点和计算描述子
    cout << "extracting features" << endl;
    compute_KeyPoints_Desp(frame1);
    compute_KeyPoints_Desp(frame2);

    // 读取相机内参
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof(paraReader.getData("camera.fx" ).c_str());
    camera.fy = atof(paraReader.getData("camera.fy" ).c_str());
    camera.cx = atof(paraReader.getData("camera.cx" ).c_str());
    camera.cy = atof(paraReader.getData("camera.cy" ).c_str());
    camera.scale = atof(paraReader.getData("camera.scale" ).c_str());

    // solving PnP
    RESULT_OF_PNP result = estimateMotion(frame1, frame2, camera);

    // Process result
    // Transform rotation vector to matrix
    cv::Mat R;
    cv::Rodrigues(result.rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 将translation + rotation matrix组合成transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    cout << "translation" << endl;
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0);
    T(1,3) = result.tvec.at<double>(0,1);
    T(2,3) = result.tvec.at<double>(0,2);

    // 转换图像为点云
    cout << "converting image to cloud" << endl;
    PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );

    // 合并点云
    cout << "combining clouds" << endl;
    PointCloud::Ptr output(new PointCloud());
    pcl::transformPointCloud(*cloud1, *output, T.matrix());
    *output += *cloud2;
    pcl::io::savePCDFile("../data/result.pcd", *output);
    cout << "Final result saved." << endl;

    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(output);
    while (!viewer.wasStopped ())
    {}

    return 0;
}

