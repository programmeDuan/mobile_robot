/******************************************************************************
Copyright (C), 2020, Yat  Co., Ltd.

File Name     : planar_segmentation.cpp
Version       : Initial Draft
Author        : @yat.com">guangxue.duan@yat.com
Created       :2020.5
Description   :Extract plane boundary lines
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

//OpenCv
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

//Eigen
#include <Eigen/Dense>

//realsense
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV

//ros
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

//c++
#include <string>
#include <iostream>

using namespace cv;
using namespace rs2;
using namespace std;

#define PLANEERROR 0.01
#define OBJECTHEIGHTMAX 0.5
#define OBJECTHEIGHTMIN 0.0

#define DETWIDTH 80 //CM
#define DETHEIGHT 80 //CM

//Camera internal reference
double fx = 424.6;
double fy = 424.6;
int cx = 418;
int cy = 243;

int element_width = 7;
int element_height = 7;

int coordinate_x = 374;
int coordinate_y = 380;
int rect_width = 100;
int rect_height = 100;

int imgout_width = 200;
int imgout_height = 200;
int showroi_width = 300;
int showroi_height = 300;

int height_threshold = 3;//CM
/*****************************************************************************
Function Ｎame: yatPointToMatrix
Description: this function is used to transform point to 4*4 matrix
Return Value: 4*4 Matrix 
Date: 2020-05-26
Author: guangxue.duan@yat.com
*****************************************************************************/
Eigen::Matrix4f yatPointToMatrix(double a,double b,double c)
{
    if(c<0)
    {
      c = 0-c;
      b = 0-b;
      a = 0-a;
    }

    double lengthVector = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    double roll = atan2(b,c); // x
    double pitch = - asin(a/lengthVector); // y
    double yaw = 0; // z

    Eigen::Matrix4f convertMatrix(4,4); // Body to Navigation
    convertMatrix(0,0) = cos(pitch)*cos(yaw);
    convertMatrix(1,0) = cos(pitch)*sin(yaw);
    convertMatrix(2,0) = -sin(pitch);

    convertMatrix(0,1) = - cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,1) = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,1) = sin(roll)*cos(pitch);

    convertMatrix(0,2) = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,2) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,2) = cos(roll)*cos(pitch);

    convertMatrix(0,3) =0;
    convertMatrix(1,3) =0;
    convertMatrix(2,3) =0;
    convertMatrix(3,3) =1;
    convertMatrix(3,0) =0;
    convertMatrix(3,1) =0;
    convertMatrix(3,2) =0;

    return convertMatrix;
}

/*****************************************************************************
Function Ｎame: yatPointToMatrix3
Description: this function is used to transform point to 3*3 matrix
Return Value: 3*3 Matrix 
Date: 2020-05-26
Author: guangxue.duan@yat.com
*****************************************************************************/
Eigen::Matrix3f yatPointToMatrix3(double a,double b,double c)
{
    if(c<0)
    {
      c = 0-c;
      b = 0-b;
      a = 0-a;
    }
    
    double lengthVector = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    double roll = atan2(b,c); // x
    double pitch = - asin(a/lengthVector); // y
    double yaw = 0; // z

    Eigen::Matrix3f convertMatrix(3,3); // Body to Navigation
    convertMatrix(0,0) = cos(pitch)*cos(yaw);
    convertMatrix(1,0) = cos(pitch)*sin(yaw);
    convertMatrix(2,0) = -sin(pitch);

    convertMatrix(0,1) = - cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,1) = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,1) = sin(roll)*cos(pitch);

    convertMatrix(0,2) = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,2) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,2) = cos(roll)*cos(pitch);

    return convertMatrix;
}

/*****************************************************************************
Function Ｎame: yatGetConvertMatrix
Description: this function is used to get 3*3 convertmatrix
Return Value: 3*3 convertmatrix 
Date: 2020-05-26
Author: guangxue.duan@yat.com
*****************************************************************************/
Eigen::Matrix3f yatGetConvertMatrix()
{
    Eigen::Matrix3f convertMatrix(3,3); // Body to Navigation
    convertMatrix(0,0) = 1/ fx;
    convertMatrix(0,2) = - cx/ fx;
    convertMatrix(1,1) = 1/ fy;
    convertMatrix(1,2) = - cx/ fy;

    convertMatrix(0,1) = 0;
    convertMatrix(1,0) = 0;
    convertMatrix(2,0) = 0;
    convertMatrix(2,1) = 0;
    convertMatrix(2,2) = 1.0;
    return convertMatrix;
}

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
              pcl::PointIndices::Ptr planeIndices)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(PLANEERROR);
  // segmentation.setMaxIterations(500);
  // segmentation.setDistanceThreshold(0.015);
  segmentation.setOptimizeCoefficients(true);
  segmentation.segment(*planeIndices, *coefficients);
}

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
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (plane);
  proj.setModelCoefficients (coefficients);
  proj.filter (*proj_cloud);
}

/*****************************************************************************
Function Ｎame: yatRemovePlane
Description:this function is used to removes either the points on the plane from the entire point cloud
            or only keeps the points on the plane.This distinction is defined by setNeg(if setNeg is true, deletes points on the plane).
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
                 bool setNeg)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setNegative(setNeg);
  extract.setInputCloud(inputCloud);
  extract.setIndices(cloudIndices);
  extract.filter(*outCloud);
}

/*****************************************************************************
Function Ｎame : yatLoadDepthImage
Description : this function is used to transform depth information into point cloud information
param depth_img : input depthimage message
return value : if not depthimage message or depthimage message type is not CV_16U,return false; otherwise,return true;
Date : 2020-05-26
Author :guangxue.duan@yat.com
*****************************************************************************/
bool yatLoadDepthImage(Mat depth_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &init_cloud, Rect roi)
{
	//depth_img = depth_img(Rect(0, 240, 848, 240));
	if (depth_img.empty() || depth_img.depth() != CV_16U)
	{
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}
	int rows = depth_img.rows, cols = depth_img.cols;
	int vertex_idx = 0;
	for (int i = 0; i < rows -4 ; i+=4)
	{
		for (int j = 0; j < cols -4; j +=4)
		{
			pcl::PointXYZRGB point;
			point.z = (float)(depth_img.at<unsigned short>(i, j)) / 1000.0;
			//cout<<point.z<< " ";
			if (point.z > 100000)
			{
				point.x = 0;
				point.y = 0;
				init_cloud->points.push_back(point);
				continue;
			}
			point.x = ((float)j + roi.x - 418) * point.z / 424.6;
			point.y = ((float)i  + roi.y - 243) * point.z / 424.6;
      point.r = 100;
      point.g = 100;
      point.b = 100;
			init_cloud->points.push_back(point);
		}
		//cout<<endl;
	}
	return true;
}

/*****************************************************************************
Function Ｎame : yatCaculateMean
Description : this function is used to calculate the mean distance of camera and PointCloud
param cloud : input PointCloud message
return value : mean distance
Date : 2020-05-26
Author :guangxue.duan@yat.com
*****************************************************************************/
float yatCaculateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  float sum = 0;
  int valnum = 0;
  for(int i=0; i<cloud->points.size(); i++)
  {
    //cout<<cloud->points[i].z<<endl;
    if(cloud->points[i].z > 0)
    {
      sum += cloud->points[i].z;
      valnum ++;
    }
    
  }
  return sum / valnum;
}

/*****************************************************************************
Function Ｎame : yatGetLine
Description : this function is used to extract the boundary line of the image
param outroi : input image message
return value : lines message
Date : 2020-05-26
Author :guangxue.duan@yat.com
*****************************************************************************/
vector<int> yatGetLine(Mat outroi)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy; 
  findContours(outroi,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);

  
  cout<<"countour num is: "<<contours.size()<<endl;
  // imshow("outroi", outroi);
  // waitKey(10);

  vector<int> lines(DETHEIGHT);

  for(int i=0; i<DETHEIGHT; i++)
  {
    lines[i] = 0;
  } 

  if(contours.size()==0)
  {
    cout<<"Do not find countour!"<<endl;
    return lines;
  }

  else if(contours.size() == 1)
  {
    Mat showcontimg = Mat::zeros(outroi.size(), CV_8UC1);
    drawContours(showcontimg,contours,0,Scalar(255),CV_FILLED,8,hierarchy);
    imshow("contour", showcontimg);

    int valid[DETHEIGHT] = {0};
    for(int i=0; i<DETHEIGHT; i++)
    {
      for(int j=0; j<DETWIDTH;j++)
      {
        valid[i] += showcontimg.at<uchar>(i, j) / 200;
      }
    }

    for(int i=20; i<DETHEIGHT; i++)
    {
      lines[i] = valid[i];
    }

    return lines;
  }
  else 
  {
    int index = 0;
    int contourid = -1;
    while(index != -1)
    {
      Rect rect = boundingRect(Mat(contours[index]));

      // cout<<"rect.x="<<rect.x<<endl;
      if(rect.x <= 0)
      {
        contourid = index;
        break;
      }
      else
      {
        index = hierarchy[index][0];
      }
    }
    if(contourid >= 0)
    {
      Mat showcontimg = Mat::zeros(outroi.size(), CV_8UC1);
      drawContours(showcontimg,contours,contourid,Scalar(255),CV_FILLED,8,hierarchy);
      imshow("contour", showcontimg);

      int valid[DETHEIGHT] = {0};
      for(int i=0; i<DETHEIGHT; i++)
      {
        for(int j=0; j<DETWIDTH;j++)
        {
          valid[i] += showcontimg.at<uchar>(i, j) / 255;
        }
      }

      for(int i=20; i<DETHEIGHT; i+=10)
      {
        lines[i] = valid[i];
      }

      return lines;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_segmentation");

  ros::NodeHandle nh;
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeProjected(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::ModelCoefficients::Ptr planeCoef(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);

  pipeline pipe;//The class abstracts the camera configuration and streaming, and the vision modules triggering and threading
  pipe.start();
  Mat element = getStructuringElement(MORPH_RECT, Size(element_width, element_height));

  ros::Publisher line_pub_ = nh.advertise<geometry_msgs::PoseArray>("planpoints", 1);

  while(ros::ok())
  {

    frameset data = pipe.wait_for_frames();
    Mat depth = frame_to_mat(data.get_depth_frame());
    Mat color = frame_to_mat(data.get_color_frame());

    plane->resize(0);
    planeProjected->resize(0);

    Mat depth_roi = depth(Rect(coordinate_x, coordinate_y, rect_width, rect_height));
    yatLoadDepthImage(depth_roi, plane, Rect(coordinate_x, coordinate_y, rect_width, rect_height));
    /* Perform plane segmentation */
    yatPlaneSeg(plane,planeCoef,planeIndices);

    // cout<<"plane_coef "<<plane_coef->values[0]<< "  " <<plane_coef->values[1] << "  "<< plane_coef->values[2]<<endl;
    // cout<<"planeindices "<<planeIndices->indices.size()<<endl;
    Eigen::Matrix4f tmat = yatPointToMatrix(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2]);

    yatRemovePlane(planeProjected,plane,planeIndices,false);
    transformPointCloud(*planeProjected, *planeProjected, tmat);
  
    float mean = yatCaculateMean(planeProjected) * 100;

    cout<<"mean is "<<mean<<endl;

    Eigen::Matrix3f ttransmat = (yatPointToMatrix3(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2])) .transpose();
    Mat cvtransmat;
    eigen2cv(ttransmat, cvtransmat);

    double t1 = getTickCount();

    Mat val= Mat(407040, 3, CV_32F);   

    for(int i = 0; i<depth.rows-2; i +=2)
    {
      for(int j=0; j<depth.cols-2; j +=2)
      {
        float z = val.at<float>(i*depth.cols + j, 2) = (float)(depth.at<unsigned short>(i, j)) / 1000;
        val.at<float>(i*depth.cols + j, 0) = (j - cx) * z / fx;
        val.at<float>(i*depth.cols + j, 1) = (i - cy) * z / fy;
      }
    }

    Mat outval = val * cvtransmat;
    // cout<<"coutval "<<val.size()<<" "<<outval.size()<<endl;
  
    Mat imgout = Mat::zeros(Size(imgout_width, imgout_height), CV_8UC1);

    for(int i = 0; i<depth.rows-2; i+=2)
    {
      for(int j = 0; j<depth.cols-2; j+=2)
      {

        int x = outval.at<float>(i*depth.cols + j, 0) * 100;
        int y = outval.at<float>(i*depth.cols + j, 1) * 100;
        float z = outval.at<float>(i*depth.cols + j, 2) * 100;
       
        if(x > -imgout_width/2 && x < imgout_width/2 && y <0 && y>-imgout_height) 
        {
          if(isnan(mean))
             continue;
          else
          {
            if(z < mean + height_threshold && z > mean -height_threshold || z == 0)
            {
              imgout.at<uchar>(-y, x + imgout_width/2) = 255;//100
            }

            else
            {
              imgout.at<uchar>(-y, x + imgout_width/2) = 100;//0.5
            }
          }
        }
      }
    }

    morphologyEx(imgout, imgout, MORPH_CLOSE, element);
    Mat outroi = imgout(Rect(100, 0, DETWIDTH, DETWIDTH)).clone();

    flip(imgout, imgout, 0);
    imshow("show", imgout);

    Mat showroi;
    resize(outroi, showroi, Size(showroi_width, showroi_height));

    flip(showroi, showroi, 0);
    imshow("showroi", showroi);
    
    outroi = outroi / 255;
    vector<int> line = yatGetLine(outroi);
    cout<<line.size()<<endl;
    // imshow("outroi", outroi);
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();

    geometry_msgs::Pose p;
    p.position.x = 25;
    p.position.y = line[25];
    cout<<"1:       "<<line[25]<<endl;
    msg.poses.push_back(p);
    p.position.x = 40;
    p.position.y = line[40];
    cout<<"1:       "<<line[40]<<endl;
    msg.poses.push_back(p);

    line_pub_.publish(msg);

    double t2 = (getTickCount() - t1) / getTickFrequency() * 1000;
    cout<<"cost time is: "<< t2 <<endl;

    waitKey(10);
  }
}