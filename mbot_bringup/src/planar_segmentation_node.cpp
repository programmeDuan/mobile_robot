#include "mbot_bringup/obstacledet.h"

//RealSense
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV

//ros
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

using namespace rs2;

// void yatSaveMap(cv::Mat mat)
// {

//   int frame_cnt =0;
//   int index =0;

//   for(int i = 0; i<= 200;i++)
//   {
//     frame_cnt = i;

//     if(frame_cnt % 200 == 0)
//     {
//     ++index;
//     frame_cnt = 0;
//     char save_file[200];
//     sprintf(save_file,"/home/yat/图片/6-2/%d.jpg",index);
//     cv::imwrite(save_file,mat);
//   }
//   }

// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_segmentation");
  ros::NodeHandle nh;
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeprojected(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::ModelCoefficients::Ptr planeCoef(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  
  // YATPlanarSegmentation planarsegmentation;
  YATPlanarSegmentation *planarsegmentation = new YATPlanarSegmentation;
  YATObstracleDet *obstracledet = new YATObstracleDet;

  pipeline pipe;//The class abstracts the camera configuration and streaming, and the vision modules triggering and threading
  pipe.start();

  int frame_cnt = 0;
  int index = 0;

  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(planarsegmentation->yatGetElementWidth(), planarsegmentation->yatGetElementHeight()));

  ros::Publisher line_pub_ = nh.advertise<geometry_msgs::PoseArray>("planpoints", 1);

  while(ros::ok())
  {

    frameset data = pipe.wait_for_frames();
    cv::Mat depth = frame_to_mat(data.get_depth_frame());
    cv::Mat color = frame_to_mat(data.get_color_frame());
    
    // yatSaveMap(color);

    if(frame_cnt % 200 == 0)
    {
      ++index;
      frame_cnt = 0;
      char save_file[200];
      sprintf(save_file,"/home/yat/图片/6-2/%d.jpg",index);
      cv::imwrite(save_file,color);
    }

    frame_cnt++;

    plane->resize(0);
    planeprojected->resize(0);

    planarsegmentation->yatGetPlaneBoundaryLine(depth,element,plane,planeprojected,planeCoef,planeIndices);

    cloud_projected = obstracledet->yatGetObstraclePointCloud(depth,plane,planeprojected,planeCoef,planeIndices,planarsegmentation);
    
    if(!obstracledet->yatIsFindObstracle(cloud_projected))
    {
      // planarsegmentation->yatGetPlaneBoundaryLine(depth,element,plane,planeprojected,planeCoef,planeIndices);

      geometry_msgs::PoseArray msg;
      msg.header.stamp = ros::Time::now();

      geometry_msgs::Pose p;
      p.position.x = 25;
      p.position.y = planarsegmentation->linepoints_[25];
      cout<<"1:       "<<planarsegmentation->linepoints_[25]<<endl;
      msg.poses.push_back(p);
      p.position.x = 40;
      p.position.y = planarsegmentation->linepoints_[40];
      cout<<"2:       "<<planarsegmentation->linepoints_[40]<<endl;
      msg.poses.push_back(p);

      line_pub_.publish(msg);
    } 

    else
    {
      geometry_msgs::PoseArray msg;
      msg.header.stamp = ros::Time::now();

      geometry_msgs::Pose p;
      p.position.x = 25;
      p.position.y = 0;
      msg.poses.push_back(p);
      p.position.x = 40;
      p.position.y = 0;
      msg.poses.push_back(p);

      line_pub_.publish(msg);
    }
    // cv::Mat depthnoplane= obstracledet->yatGetObstracleDepth(depth,plane,planeprojected,planeCoef,planeIndices,planarsegmentation);
    // obstracledet->yatFindObstacle(depthnoplane,obstracledet->yatGetThresh(),obstracledet->yatGetMaxThresh(),obstracledet->yatGetArea());
  }

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->initCameraParameters ();
  // int v1(0);
  // viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  // viewer->setBackgroundColor (0, 0, 0, v1);
  // viewer->addCoordinateSystem();
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (plane);
  // viewer->addPointCloud<pcl::PointXYZRGB> (plane, rgb, "sample cloud1", v1);
  // int v2(0);
  // viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  // viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  // viewer->addCoordinateSystem();
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (cloud_projected, 0, 255, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud_projected, single_color, "sample cloud2", v2);
  // while (!viewer->wasStopped())
  // {
	// 	viewer->spinOnce(100);
	// 	usleep(100000);
	// }

  delete obstracledet;
  delete planarsegmentation;
  obstracledet = NULL;
  planarsegmentation = NULL;
  
  // return 0;
}
