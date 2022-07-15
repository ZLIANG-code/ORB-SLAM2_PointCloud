/*
 * 实时点云构建
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <condition_variable>
 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include<pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include<pcl/filters/passthrough.h>
 #include<pcl/filters/voxel_grid.h>
 
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

// PCL_INSTANTIATE(Search, PCL_POINT_TYPES)

namespace ORB_SLAM2
{

class PointCloudMapping
{
public:
    // 定义点云对象
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // INFO
    mutex               IsLoopMutex;
    bool bIsLoop = false;


    PointCloudMapping( double resolution_ );
    ~PointCloudMapping();
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    // static void viewerPsycho (pcl::visualization::PCLVisualizer& viewer);
    void shutdown();
    void viewer();
    void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap);
    void reset();
    void setIsLoop(bool isLoop);
    bool getIsLoop();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    // 指针类模板创建点云对象
    PointCloud::Ptr globalMap;
    PointCloud::Ptr Part_tem;//局部地图,用于大尺度场景下的重建
    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false; // 程序退出标志位
    mutex   shutDownMutex;  
    
    bool    dataupdate    =false;       //数据更新标志位
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs,depthImgs;
    cv::Mat   depthImg,colorImg,mpose;
 
  
    vector<PointCloud::Ptr>   mvPointClouds; //存储点云序列
    vector<PointCloud::Ptr>   mvPointCloudsForMatch;
    vector<cv::Mat>   mvPosePointClouds;
    unsigned long int  mpointcloudID=0;
    
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    //显示精度
    double resolution = 0.04;
    // 体素滤波对象
    pcl::VoxelGrid<PointT>  voxel; 
    pcl::VoxelGrid<PointT>  voxelForMatch;//用于滤波得到ＩＣＰ匹配的点云
    float cx=0,cy=0,fx=0,fy=0;
    
    g2o::SparseOptimizer globaloptimizer;
};
}
#endif // POINTCLOUDMAPPING_H
