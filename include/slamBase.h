/*********************
 * note: to declare all basic functions in RBGD-SLAM tutorial
 *********************/

# pragma once

// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>



// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    /* data */
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

// 函数接口
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS camera);

void compute_KeyPoints_Desp(FRAME& frame);

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);

Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);

PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera);

CAMERA_INTRINSIC_PARAMETERS getDefaultCamera();

// 参数读取类
class ParameterReader
{
public:
    // Define data
    map<string, string> data;

    // Constructor
    ParameterReader( string filename="../parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    // Data getter
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

};

// 自定义全局static数值
static int pointCloud_count = 0;
