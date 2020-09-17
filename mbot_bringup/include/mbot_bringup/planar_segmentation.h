/******************************************************************************
Copyright (C), 2020, Yat  Co., Ltd.

File Name     : planar_segmentation.cpp
Version       : Initial Draft
Author        : @yat.com">guangxue.duan@yat.com
Created       :2020.5.28
Description   :Extract plane boundary lines 
修改记录： 2020-6-3 增加了yatPointCloudToRangeImage函数,实现将点云信息转化为深度图像
/******************************************************************************/
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

#include <pcl/range_image/range_image.h>    //关于深度图像的头文件
#include <pcl/visualization/range_image_visualizer.h>   //深度图可视化的头文件
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

// //OpenCV
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

//Eigen
#include <Eigen/Dense>

//c++
#include <string>
#include <iostream>

using namespace std;

class YATPlanarSegmentation
{
public:
    
    //友元类
    friend class YATObstracleDet;
    YATPlanarSegmentation();
    ~YATPlanarSegmentation();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the width of element
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetElementWidth();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the height of element
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetElementHeight();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the variable of height_threshold_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetHeightThreshold();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the camera internal parameter matrix parameter fx_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/ 
    float yatGetFx();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the camera internal parameter matrix parameter fy_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    float yatGetFy();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the camera internal parameter matrix parameter cx_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    float yatGetCx();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the camera internal parameter matrix parameter cy_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    float yatGetCy();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the variable of coordinate_x_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetCoordinateX();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the variable of coordinate_y_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetCoordinateY();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the variable of rect_width_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetRectWidth();

    /*****************************************************************************
    Function Ｎame: yatGetDistanceThresh
    escription: this function is used to get the variable of rect_height_
    Date: 2020-05-29
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    int yatGetRectHeigth();

    /*****************************************************************************
    Function Ｎame: yatPointToMatrix
    Description: this function is used to transform point to 4*4 matrix
    param a,b,c:is the input point
    Return Value: 4*4 Matrix 
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    Eigen::Matrix4f yatPointToMatrix(double a,double b,double c);

    /*****************************************************************************
    Function Ｎame: yatPointToMatrix3
    Description: this function is used to transform point to 3*3 matrix
    param a,b,c:is the input point
    Return Value: 3*3 Matrix 
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    Eigen::Matrix3f yatPointToMatrix3(double a,double b,double c);

    /*****************************************************************************
    Function Ｎame: yatGetConvertMatrix
    Description: this function is used to get 3*3 convertmatrix
    Return Value: 3*3 convertmatrix 
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    Eigen::Matrix3f yatGetConvertMatrix();

    /*****************************************************************************
    Function Ｎame: yatPlaneSeg
    Description: this function is used to extract plane models in point cloud.
    param cloud: the input PointCloud
    param coefficients: stores the coefficients of the output plane model
    param planeIndices: stores points that lie on the newly found plane
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    void yatPlaneSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
              pcl::ModelCoefficients::Ptr coefficients,
              pcl::PointIndices::Ptr planeIndices);
    
    /*****************************************************************************
    Function Ｎame: yatProjectPoints
    Description: this function is used to projects object points onto tabletop plane defined in coefficients
    param plane: the input PointCLoud
    param coefficients: stores the coefficients of the output plane model
    param proj_cloud: the output plane model
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    void yatProjectPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_cloud,
                   pcl::ModelCoefficients::Ptr coefficients,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane);
    
    /*****************************************************************************
    Function Ｎame: yatRemovePlane
    Description:this function is used to removes either the points on the plane from the entire point cloud
               or only keeps the points on the plane.This distinction is defined by setNeg(if setNeg is true, 
               deletes points on the plane).
    param outCloud: the output PointCloud
    param intCloud: the input PointCloud
    param cloudIndices: stores points that lie on the newly found PointCloud
    param setNeg: if setNeg is true,deletes points on the plane,else deletes points out of the plane
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    void yatRemovePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                 pcl::PointIndices::Ptr cloudIndices,
                 bool setNeg);
    /*****************************************************************************
    Function Ｎame: yatLoadDepthImage
    Description: this function is used to transform depth information into point cloud information
    param depth_img: the input depthimage message
    param init_cloud: the output PointClound message
    param roi: Window roi area 
    return value: if not depthimage message or depthimage message type is not CV_16U,return false; otherwise,return true;
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    bool yatLoadDepthImage(cv::Mat depth_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &init_cloud, cv::Rect roi);
    /***********************************************************
    函数名称：yatPointCloudToRangeImage
    函数功能：将点云信息转化为深度图像
    入口参数：input_cloud:输入点云的指针
    出口参数：range_image:输出深度图像的指针
    备 注：
    ***********************************************************/
    void yatPointCloudToRangeImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,pcl::RangeImage::Ptr range_image);
    /*****************************************************************************
    Function Ｎame: yatCaculateMean
    Description: this function is used to calculate the mean distance of camera and PointCloud
    param cloud: input PointCloud message
    return value: mean distance
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    float yatCaculateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    /*****************************************************************************
    Function Ｎame: yatGetLine
    Description: this function is used to extract the boundary line of the image
    implementation process:call the function of findContours extracting the image contours, Use the 
                           number of contours as the judgment condition,next call the drawContours 
                           to redraw the contours,then extract the boundary line of the new contours,
                           finally store the points on the line into the vector and return the vector
    param outroi: input image message
    return value: lines message
    Date: 2020-05-26
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    vector<int> yatGetLine(cv::Mat outroi);
    /*****************************************************************************
    Function Ｎame:yatGetPlaneBoundaryLine
    Description: this function is used to get the boundary of the plane by detecting the plane and extract the boundary line
    implementation process: Set ROI of depth image and get corresponding point cloud information,using point cloud information
                            and plane extraction function to extract plane and point in plane, then store in new point cloud.
                            Perform a rigid body transformation on the new point cloud and calculate the average distance from 
                            the camera to the point cloud,and then Find the transformation matrix, use this transformation matrix
                            to convert the depth information into plane information,Create a new Mat object, use the plane information
                            and the average distance and distance assignment to determine the conditions, and assign a value to this new 
                            Mat object,Close the operation and roi settings of this object,call the function of yatGetLine extracting 
                            line to get points on the line.
    param depth: the depth image(map)
    param element:structural elements of morphological operations
    param plane: the input PointCloud
    param planeProjected: New PointCloud obtained from the processing of PointCloud plane 
    param planeCoef: stores the coefficients of the output plane model
    param planeIndices: stores points that lie on the newly found plane
    Date: 2020-05-27
    Author: guangxue.duan@yat.com
    *****************************************************************************/
    void yatGetPlaneBoundaryLine(cv::Mat depth,cv::Mat element,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeProjected,
                         pcl::ModelCoefficients::Ptr planeCoef,
                         pcl::PointIndices::Ptr planeIndices);
    
    vector<int>linepoints_;//存储直线坐标
private:
    float plane_error_;//分割平面的距离阈值
    
    //用来边界提取的图像的尺寸
    int outroi_width_;
    int outroi_height_;

    float fx_,fy_,cx_,cy_;//相机内参
    
    int element_width_,element_height_;//结构元素尺寸

    //rect 参数
    int coordinate_x_,coordinate_y_,rect_width_,rect_height_;
    
    int imgout_width_,imgout_height_;
    int showroi_width_,showroi_height_;

    int height_threshold_;//高度阈值
};

