#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

// openCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>


// 给定index，读取一帧数据
FRAME readFrame(int index, ParameterReader& pd);
FRAME readFrameDepthImg(int index, ParameterReader& pd, ImgReader& imgRd, DepthReader& depthRd);
// 度量运动的大小
double normofTransform(cv::Mat rvec, cv::Mat tvec);

int main(int argc, char** argv)
{
    ParameterReader pd;
    ImgReader imgRd;
    DepthReader depthRd;
    // c_str()函数返回一个指向正规C字符串的指针常量，内容与本string串相同。
    // 这是为了与C语言兼容，在C语言中没有string类型
    // 故必须通过string类对象的成员函数c_str()把string对象转换成C中的字符串样式。
    // 如果一个函数要求char*，则可以用c_str()
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    //initialize
    cout << "Initializing ..." << endl;
    int currIndex = startIndex; // 当前索引为currIndex
    //FRAME lastFrame = readFrame(currIndex, pd); //上一帧数据
    FRAME lastFrame = readFrameDepthImg(currIndex, pd, imgRd, depthRd); //上一帧数据
    // 我们总是在比较currFrame和lastFrame
    // 这个getDefaultCamera()在哪里定义的哦？ 在slamBase.h
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    compute_KeyPoints_Desp(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    // 是否显示点云
    pcl::visualization::CloudViewer viewer("cloud viewer");
    bool visualize = pd.getData("visualize_pointcloud") == string("yes");

    int min_inliers = atoi(pd.getData("min_inliers").c_str());
    double max_norm = atof(pd.getData("max_norm").c_str());

    for(currIndex = startIndex+1; currIndex<endIndex; currIndex++)
    {
        cout << endl;
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrameDepthImg(currIndex, pd, imgRd, depthRd); //读取currFrame
        compute_KeyPoints_Desp(currFrame);
        // 比较 [currFrame] and [lastFrame]
        RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame,camera);
        if(result.inliers < min_inliers)
        {
            // 如果inliers不够，放弃该帧
            cout << "inliers not enough, abandoning this frame" << endl;
            continue;
        }
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout << "norm = " << norm << endl;
        if(norm >= max_norm)
        {
            cout << "normal out of range" << endl;
            continue;
        }
            
        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
        // cout << "T = " << T.matrix() << endl;

        cloud = joinPointCloud(cloud,currFrame, T,camera);

        if(visualize == true)
        {
            cout << "showing cloud" << endl;
            viewer.showCloud(cloud);
        }

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile("../data/result/result.pcd", *cloud);
    return 0;
}

FRAME readFrame(int index, ParameterReader& pd)
{
    FRAME f;

    string rgbDir = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");

    string rgbExt = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    ss << rgbDir << index << rgbExt;
    string filename;
    ss >> filename;
    f.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();
    ss << depthDir << index << depthExt;
    ss >> filename;
    f.depth = cv::imread(filename, -1);

    return f;
}

FRAME readFrameDepthImg(int index, ParameterReader& pd, ImgReader& imgRd, DepthReader& depthRd)
{
    FRAME f;

    string rgbDir = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");

    string rgbExt = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    string filename;

    cout << "img_index: " << imgRd.getImg(index) << endl;
    ss << rgbDir << imgRd.getImg(index) << rgbExt;
    ss >> filename;
    f.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();

    cout << "depth_index: " << depthRd.getDepth(index) << endl;
    ss << depthDir << depthRd.getDepth(index) << depthExt;
    ss >> filename;
    f.depth = cv::imread(filename, -1);

    return f;
}

double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}
