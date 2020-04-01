#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

// openCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

// g2o 库
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// 给定index，读取一帧数据
FRAME readFrame(int index, ParameterReader& pd);
// 度量运动的大小
double normofTransform(cv::Mat rvec, cv::Mat tvec);

// 选择优化方法, 把g2o的定义放到前面
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

int main(int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    //initialize
    cout << "Initializing ..." << endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME lastFrame = readFrame(currIndex, pd); //上一帧数据
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

    /********
     * 新增：有关g2o的初始化
    ********/

    // 初始化求解器
    // SlamLinearSolver* linearSolver = new SlamLinearSolver();
    std::unique_ptr<SlamBlockSolver::LinearSolverType> linearSolver(new SlamLinearSolver);

    //linearSolver->setBlockOrdering(false);
    // SlamBlockSolver* blockSolver = new SlamBlockSolver(unique_ptr<SlamBlockSolver::LinearSolverType>(linearSolver));
    std::unique_ptr<SlamBlockSolver> blockSolver(new SlamBlockSolver(std::move(linearSolver)));

    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    g2o::SparseOptimizer globalOptimizer; //最后用的就是这个东东
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);    //不输出调试信息

    // 向globalOptimizer增加第一个vertex
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity());  //估计为单位矩阵
    v->setFixed(true);                              //第一个顶点固定，不用优化
    globalOptimizer.addVertex(v);

    int lastIndex = currIndex;                      //上一帧的id

    for(currIndex = startIndex+1; currIndex<endIndex; currIndex++)
    {
        cout << endl;
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd); //读取currFrame
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

        //cloud = joinPointCloud(cloud,currFrame, T,camera);

        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(currIndex);
        v->setEstimate(Eigen::Isometry3d::Identity());
        globalOptimizer.addVertex(v);
        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices()[0] = globalOptimizer.vertex(lastIndex);
        edge->vertices()[1] = globalOptimizer.vertex(currIndex);
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵的是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息矩阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        //也可以将角度设大一些，对角度的估计就更加准确
        edge->setInformation(information);
        // 边的估计，即是pnp求解之结果
        edge->setMeasurement(T);
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;

    }

    //pcl::io::savePCDFile("../data/result/result.pcd", *cloud);

    // 优化所有边
    cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.save("../data/g2o_result/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);  //  可以指定优化步数
    globalOptimizer.save("../data/g2o_result/result_after.g2o");
    cout << "optimization done." << endl;

    globalOptimizer.clear();

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

double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}
