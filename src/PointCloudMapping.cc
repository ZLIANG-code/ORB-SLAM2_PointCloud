/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
#include <chrono>
#include <ctime>
#include <climits>

#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"

#include "PointCloudMapping.h"
 
namespace ORB_SLAM2
{

/*
 * @ 设置点云分辨率
 */
PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    // this->resolution =0.005;
    voxel.setLeafSize( resolution, resolution, resolution);
    voxelForMatch.setLeafSize( 0.1,0.1, 0.1);
    // 点云对象（指针）初始化
    globalMap = boost::make_shared< PointCloud >( );
    Part_tem = boost::make_shared< PointCloud >( );

    lastKeyframeSize=0;

    // 开启绘图线程
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

PointCloudMapping::~PointCloudMapping()
{
     viewerThread->join();
}

// 复位点云显示模块
void PointCloudMapping::reset()
{
      mvPosePointClouds.clear();
      mvPointClouds.clear();
      mpointcloudID=0;
}

// 保存生成点云文件
void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
    }
 
    string save_path = "./pcd_file/pointCloud.pcd";
    pcl::io::savePCDFile(save_path,*globalMap);
    cout<<"save pcd files to :  "<<save_path<<endl;
}

// 得到点云创建所需参数：关键帧、颜色、深度、相机外参
// 由外部函数调用，每生成一个关键帧调用一次该函数
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    unique_lock<mutex> lck(keyframeMutex);
    cv::Mat T =kf->GetPose();
    keyframes.push_back(kf);
    // 得到每个点云的pose
    mvPosePointClouds.push_back(T.clone());    
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    if(cx ==0 || cy ==0||fx==0||fy==0)
    {
        cx = kf->cx;
        cy = kf->cy;
        fx = kf->fx;
        fy = kf->fy;
    }
    // NOTE: notify_one用来唤醒阻塞的线程
    keyFrameUpdated.notify_one();
    // cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
}

/*
 * 根据关键帧生成点云
 */
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    // 遍历关键帧像素
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;  
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
            
            tmp->points.push_back(p);
        }
    }
  
    // rotation the pointcloud and stiching 
    // 初始化变换矩阵对象
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    // 初始化点云对象
    PointCloud::Ptr cloud(new PointCloud);
    // NOTE:transformPointCloud进行点云变换：tmp:为源点云、cloud为变换后的点云、第三个参数为变换矩阵
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());

    // TODO:翻转、平移处理
    // 绕x轴旋转135度，将z轴指向上方
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-M_PI*3/4, Eigen::Vector3f(1,0,0)));
    // 沿着z轴平移
    transform.translation() << 0.0, 0.0, 2.5;
    pcl::transformPointCloud(*cloud, *cloud, transform);



    cloud->is_dense = false;
    
    // 体素滤波
    // 设置输入点云
    voxel.setInputCloud( cloud );
    // 执行滤波，结果保存在tmp中
    voxel.filter( *tmp );
    // tmp赋值给cloud
    cloud->swap( *tmp );     
    // cout<<"generate point cloud from  kf-ID:"<<kf->mnId<<", size="<<cloud->points.size()<<endl;

    return cloud;
}

// TODO:优化点云显示方法
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        // INFO：如果检测到回环，则重新绘制所有点云地图，如果没有，则仅绘制新增的关键帧
        if(bIsLoop)
        {
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif

            printf("检测到回环，现在重新绘制所有的关键帧！\n");
            globalMap->clear();
            // 将全部关键帧插入全局地图中
            for( size_t i=0; i<N ; i++ )
            {

                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
            }

            #ifdef COMPILEDWITHC11
                     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
            #endif

            // double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            cout << "重新产生所有的关键帧点云需要的时间是：" << ttrack << endl;
            
            bIsLoop = false;
        }
        else
        {
            // 将新增的关键帧插入全局地图中
            for( size_t i=lastKeyframeSize; i<N ; i++ )
            {
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
            }
        }

        viewer.showCloud(globalMap);
        // cout<<"show global map, size="<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
        cout<<"Key frame numbers:="<<lastKeyframeSize<<endl;
    }
}

// 获取全局点云地图
void PointCloudMapping::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
{
	   unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );   
	   outputMap= globalMap;
}

void PointCloudMapping::setIsLoop(bool isLoop)
{
	   unique_lock<mutex> lck_keyframeUpdated(IsLoopMutex);   
	   bIsLoop = false;
}

bool PointCloudMapping::getIsLoop()
{
	   unique_lock<mutex> lck_keyframeUpdated(IsLoopMutex);   
	   return bIsLoop;
}

// /*
//  * 
//  * 原来版本的显示函数
//  * 由于随着尺寸的增加以后,显示函数会异常退出
//  * // INFO:在这个函数里实现点云的拼接，有点麻烦
//  */
// void PointCloudMapping::viewer()
// {
//     // 创建点云显示对象
//     pcl::visualization::CloudViewer viewer("viewer");
//     while(1)
//     {
//         // 如果shutdown信号发出，则退出
//         {
//             unique_lock<mutex> lck_shutdown( shutDownMutex );
//             if (shutDownFlag)
//             {
//                 break;
//             }
//         }
//         {
//             unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
//             keyFrameUpdated.wait( lck_keyframeUpdated );
//         }
        
//         // keyframe is updated 
//         // 这种方式下每次都需要吧队列里面所有的关键帧都去生成点云，效率不高
//         // TODO:尝试将之前生成的点云存储到向量中，然后新传入的点云加入，不每次都遍历所有
//         size_t N=0,i=0;
//         {
//             unique_lock<mutex> lck( keyframeMutex );
//             N =mvPosePointClouds.size();
//         }
//         for(i=lastKeyframeSize; i<N ; i++ )
//         {
//             //PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
//             //*globalMap += *p;
//             if((mvPosePointClouds.size() != colorImgs.size() )|| (mvPosePointClouds.size()!= depthImgs.size() ) || (depthImgs.size() != colorImgs.size() ))
//             {
//                 cout<<" depthImgs.size != colorImgs.size()  "<<endl;
//                 continue;
//             }
//             cout<<"i: "<<i<<"  mvPosePointClouds.size(): "<<mvPosePointClouds.size()<<endl;
            
//             PointCloud::Ptr tem_cloud1(new PointCloud());
//             PointCloud::Ptr tem_cloud2(new PointCloud);
//             tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i]); //生成一幅点云大约在０．１s左右
            
//             Eigen::Isometry3d T_c2w =ORB_SLAM2::Converter::toSE3Quat( mvPosePointClouds[i]);
            
//             Eigen::Isometry3d T_cw= Eigen::Isometry3d::Identity();
//             if(mvPointClouds.size()>1)
//             {
//                     Eigen::Isometry3d T_c1w =ORB_SLAM2::Converter::toSE3Quat( mvPosePointClouds[i-1]);
//                     Eigen::Isometry3d T_c1c2 = T_c1w*T_c2w.inverse();// T_c1_c2
                                
//                     PointCloud::Ptr tem_match_cloud1 =mvPointCloudsForMatch[i-1];
//                     PointCloud::Ptr tem_match_cloud2 =mvPointCloudsForMatch[i];
        
//                     computeTranForTwoPiontCloud2(tem_match_cloud1,tem_match_cloud2,T_c1c2);// 计算cloud1 cloud2 之间的相对旋转变换
                    
//                     T_cw = T_c1c2*T_c1w;
//             }
 
//  		pcl::transformPointCloud( *tem_cloud1, *tem_cloud2, T_c2w.inverse().matrix());
//  		//pcl::transformPointCloud( *tem_cloud1, *tem_cloud2,T_cw.inverse().matrix());
		
//  		*globalMap += *tem_cloud2;
// 	}
// 	lastKeyframeSize = i;
// 	viewer.showCloud(globalMap);
//     cout<<"show global map, size="<<globalMap->points.size()<<endl;	
// 	}
// }

// -----end of namespace
}
