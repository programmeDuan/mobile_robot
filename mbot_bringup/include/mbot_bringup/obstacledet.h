/******************************************************************************
Copyright (C), 2020, Yat  Co., Ltd.

File Name     : ObstracleDet.h
Version       : Initial Draft
Author        : @yat.com">guangxue.duan@yat.com
Created       :2020.5.28
Description   :detect obstracle  
/******************************************************************************/
#include "mbot_bringup/planar_segmentation.h"
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

//OpenCV
#include "opencv2/opencv.hpp"

//c++
#include <string>
#include <iostream>

using namespace std;

class YATObstracleDet
{
public:
    YATObstracleDet();
    // YATObstracleDet(const int distance_threshold,const int thresh,const int max_thresh,const int area);
    ~YATObstracleDet();
    
    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get distance_threshold_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetDistanceThresh();

    /*****************************************************************************
    Function Ｎame: yatGetThresh
    escription: this function is used to get the Binarized parameter of thresh_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetThresh();

    /*****************************************************************************
    Function Ｎame: yatGetThresh
    escription: this function is used to get the Binarized parameter of max_thresh_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetMaxThresh();

    /*****************************************************************************
    Function Ｎame: yatGetThresh
    escription: this function is used to get the variable of area_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetArea();

    /*****************************************************************************
    Function Ｎame: yatGetObstraclePointCloud
    escription: this function is used to Obtain the PointCloud information of the obstacle after removing the ground plane
    param depth: the input depth image
    param plane: the input PointCloud
    param planeProjected: New PointCloud obtained from the processing of PointCloud plane 
    param planeCoef: stores the coefficients of the output plane model
    param planeIndices: stores points that lie on the newly found plane
    param planarseg: the objects of class YATPlanarSegmentation
    return value: Pointer of obstacle point cloud
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  yatGetObstraclePointCloud(cv::Mat depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeProjected,
                         pcl::ModelCoefficients::Ptr planeCoef,
                         pcl::PointIndices::Ptr planeIndices,
                         YATPlanarSegmentation *planarseg);

    /*****************************************************************************
    Function Ｎame: yatIsFindObstracle
    escription: this function is used to  detect the presence of obstacles
    param depth: the input obstacle pointcloud message 
    return value: If the size of obstacle point cloud is less than 50, return false; when 
                  the size of obstacle point cloud is more then 50,if the coordinates x 
                  and y of the point cloud meet a certain threshold at the same time, 
                  return true, otherwise return false;
    Date: 2020-06-01
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    bool yatIsFindObstracle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);

    cv::Mat yatPointCloudToDepth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);

    /*****************************************************************************
    Function Ｎame: yatMaskDepth
    Description: this function is used to be binarization of depth map
    param depth: is the input depth image
    param outimg: is the output image after binarization
    param threshold: is the maximum distance, unit is mm, greater than this distance is safe, do not need to consider.
    Date: 2020-05-27
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    void yatMaskDepth(cv::Mat depth, cv::Mat outimg,int threshold);

    /*****************************************************************************
    Function Ｎame: yatFindObstacle
    Description: this function is used to detect obstacles
    implementation process: First, through binarization, the safety distance of one meter away is set to zero, next 
                            threshold to reduce the noiseand,and then calculate the convex hull of all obstacles and 
                            the area of the convex hull. When the area is less than a certain threshold, it is not 
                            considered and the obstacle is finally output The convex hull coordinates of the object.
    param depth: the input depth image
    param thresh: binarization parameter 
    param max_thresh: binarization parameter
    param area: convex hull minimum effective area 
    Date: 2020-05-27
    Author: guangxue.duan@yat.com   
    *****************************************************************************/
    vector<vector<cv::Point>> yatFindObstacle(cv::Mat depth,int thresh,int max_thresh,int area);

private:
    int distance_threshold_;//安全距离阈值

    //二值化参数
    int thresh_;
    int max_thresh_;

    int area_;//凸包最小有效面积

    float camera_height_; 

    vector<vector<cv::Point>> result_;//存储凸包的坐标
};
