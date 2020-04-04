/*************************************************************************
    > File Name: src/slamBase.cpp
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
    > Created Time: 2015年07月18日 星期六 15时31分49秒
 ************************************************************************/

#include "slamBase.h"

// OpenCV 库
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

// Eigen 库
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;


PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
    {
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    }

    // 设置并返回点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    pointCloud_count++;
    // cout << "pointCloud count: " << pointCloud_count << endl;
    // cout<<"point cloud size = "<<cloud->points.size()<<endl;

    //保存每一个点云
    // stringstream s_stream;
    // string folder_path = "../data/clouds/";
    // s_stream << folder_path << pointCloud_count << ".pcd";
    // string cloud_FileName;
    // s_stream >> cloud_FileName;
    // pcl::io::savePCDFile(cloud_FileName, *cloud);

    return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS camera)
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

void compute_KeyPoints_Desp(FRAME& frame)
{
    // 构建SURF特征提取器
//    int minHessian=400;
//    cv::Ptr<cv::xfeatures2d::SURF> _detector = cv::xfeatures2d::SURF::create(minHessian);
    cv::Ptr<cv::ORB> _detector = cv::ORB::create();

    if(!_detector)
    {
        cout << "Unknown detector type!" << endl;
    }

    // 提取关键点和计算描述子
    _detector->detectAndCompute(frame.rgb, cv::Mat(), frame.kp, frame.desp);

    // 可视化， 显示关键点
    // cv::Mat imgShow;
    // cv::drawKeypoints( frame.rgb, frame.kp, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::imshow( "keypoints", imgShow );
    // cv::waitKey(0); //暂停等待一个按键
}

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader paraRead;
 
    // 匹配描述子
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    vector<cv::DMatch> matches;
    matcher->match( frame1.desp, frame2.desp, matches );
    // cv::FlannBasedMatcher matcher;
    // vector< cv::DMatch > matches;
    // matcher.match( frame1.desp, frame2.desp, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 可视化：显示匹配的特征
    // cv::Mat imgMatches;
    // cv::drawMatches( frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, matches, imgMatches );
    // cv::imshow( "matches", imgMatches );
    // cv::imwrite( "../data/matches.png", imgMatches );
    // cv::waitKey( 0 );

    // 筛选匹配，把距离太大的去掉
    RESULT_OF_PNP result_pnp;   //pnp计算结果： rvec, tvec, inlier
    vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof(paraRead.getData("good_match_threshold").c_str());
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }
    // 显示 good matches
    cout << "good matches=" << goodMatches.size() << endl;
    // cv::drawMatches( frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, goodMatches, imgMatches );
    // cv::imshow( "good matches", imgMatches );
    // cv::imwrite( "../data/good_matches.png", imgMatches );
    // cv::waitKey(0);

    if (goodMatches.size() <= 7) 
    {
        result_pnp.inliers = -1;
        return result_pnp;
    }
    // // 下面这个else if是夏自己添加的
    // else if(goodMatches.size() == 500)
    // {
    //     result_pnp.inliers = -1;
    //     return result_pnp;
    // }

    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    cout << "pushing good matches " << endl;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        // TODO: DEBUG
        cout << "1";
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        cout << "2" << endl;
        cout << "y: " << int(p.y) << " x: " <<int(p.x) << endl;
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        cout << "3";
        if (d == 0)
        {
            cout << "4";
            continue;
        }
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );
        cout << "5";
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cout << "6";
        cv::Point3f pd = point2dTo3d( pt, camera );
        cout << "7" << endl;
        pts_obj.push_back( pd );
    }
    cout << "finishing push of good matches " << endl;

    double camera_matrix_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };

    cout << "solving PnP" << endl;
    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;

    // 求解pnp
    ParameterReader paraRd;
    double confidence = stod(paraRd.getData("confidence"));
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 0.9, confidence, inliers );


    result_pnp.rvec = rvec;
    result_pnp.tvec = tvec;
    result_pnp.inliers = inliers.rows;

    cout<<"inliers: "<<inliers.rows<<endl;
    // cout<<"R="<<rvec<<endl;
    // cout<<"t="<<tvec<<endl;

    return result_pnp;

}

// cvMat2Eigen: 将cv的旋转矢量和位移矢量转换为变换矩阵,类型为Eigen:Isometry3d
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R); // cv::Rodrigues(src, dst)可以将旋转向量转换为旋转矩阵
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);  // cv::cv2eigen()转换为eigen类型

    // 将rvec和tvec转为成变换矩阵
    Eigen::Isometry3d  T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    // 下面这个Translation class不知道是起到什么作用
    Eigen::Translation<double, 3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1),
            tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);

    return T;
};

// joinPointCloud
// 输入：原始点云，新来的帧,以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

    //  合并点云
    PointCloud::Ptr output(new PointCloud());
    pcl::transformPointCloud(*original, *output, T.matrix());
    *newCloud += *output;

    //  Voxel grid滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof(pd.getData("voxel_grid").c_str());
    voxel.setLeafSize(gridsize, gridsize, gridsize);
    voxel.setInputCloud(newCloud);
    PointCloud::Ptr tmp(new PointCloud());
    voxel.filter(*tmp);
    return tmp;
}
