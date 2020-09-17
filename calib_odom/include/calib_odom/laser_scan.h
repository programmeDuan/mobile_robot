#ifndef __LASER_SCAN_H__
#define __LASER_SCAN_H__
//ros//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"

#include <string>
#include "stdio.h"
#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "../include/calib_odom/Odom_Calib.hpp"
#include <csm/csm_all.h>

using namespace std;
using namespace boost::asio;//定义一个命名空间，用于后面的读写操作
//using namespace karto;

//用来进行里程计矫正的类//
extern YATOdomCalib Odom_calib;

/*
 * 获取激光数据类
*/
class YATScan
{
public:
    YATScan();

    //进行PI-ICP需要的变量//
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    //odom & scan　进行位姿积分//
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    Eigen::Vector3d now_pos,last_pos;

    //用来储存两帧之间的里程计的增量//
    std::vector<Eigen::Vector3d> odom_increments;          
    //坐标系//
    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;
    //订阅和发布的话题//
    ros::Subscriber calib_flag_sub_;
    ros::Publisher odom_path_pub_,scan_path_pub_,calib_path_pub_;

    nav_msgs::Path path_odom,path_scan;
    ros::Time current_time;
    //进行时间同步//
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    //回调函数//
    void yatCalibFlagCallBack(const std_msgs::Empty &msg);
    void yatScanCallback(const sensor_msgs::LaserScan::ConstPtr&scan2);
    //tf树查询里程计位姿-得到时刻t时,机器人在里程计坐标下的坐标//
    bool yatGetOdomPose(Eigen::Vector3d& pose, const ros::Time& t);
    //发布odom & laser path//
    void yatPublishPath(Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_);
    //发布矫正后的路径(correct path)//
    void yatPublishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_);
    //求解得到两帧数据之间的位姿差//
    //即求解当前位姿　在　上一时刻　坐标系中的坐标//
    Eigen::Vector3d yatCalDeltaDistance(Eigen::Vector3d odom_pose);
    //进行pl-icp的相关函数//
    //设置ICP参数//
    void yatSetPIICPParams();
    //把激光雷达数据 转换为PI-ICP需要的数据//
    void yatLaserScanToLDP(sensor_msgs::LaserScan *pScan,LDP& ldp);
    //求两帧之间的icp位姿匹配//                             
    Eigen::Vector3d yatPIICPBetweenTwoFrames(LDP& currentLDPScan,Eigen::Vector3d tmprPose);
};
#endif