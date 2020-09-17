/***********************************************************
文件名称：move_along_edge
作 者： 段广学
版 本：　０.0.1
说 明：控制小车沿边行走,即按订阅话题接收到的直线信息,控制小车沿着直线行走
修改记录：
***********************************************************/
#ifndef MOVE_ALONG_EDGE_H
#define MOVE_ALONG_EDGE_H
//ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "std_msgs/String.h"

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

#define PI 3.1415926
#define DegreeToRad 3.1415926535897932384626433/180 //角度转弧度
#define RadToDegree 180/3.1415926535897932384626433 //弧度转角度

//PointXY 位置结构体//
struct YATPointXY
{
    double x;
    double y;
};
//PointXYZ 位姿结构体//
struct YATPointXYZ
{
    double x;
    double y;
    //角度//
    double z;
};

//Line 直线结构体//
struct YATLine
{
    //起始点//
    YATPointXY startPoint;
    //结束点//
    YATPointXY endPoint;
};

//OdomData里程计数据结构体//
struct YATOdomData
{
    //线速度//
    double linearVel;
    //角速度//
    double angularVel;
};

class YATMoveAlongEdge
{
public:
    YATMoveAlongEdge();
    ~YATMoveAlongEdge();

    /***********************************************************
    函数名称：yatNormalAngle
    函数功能：将角度归一化在-PI到PI之间
    入口参数：需要归一化的角度(指弧度)
    出口参数：无
    返回值: 返回值为归一化后的角度
    备 注：
    ***********************************************************/
    double yatNormalAngle(double angle);
    /***********************************************************
    函数名称：yatDeltaAngle
    函数功能：计算目标角度与当前角度的归一化后的角度差值
    入口参数：targetAngle:目标角度 currentAngle:当前角度
    出口参数：无
    返回值: 返回值为归一化后的角度差值
    备 注：
    ***********************************************************/
    double yatDeltaAngle(double targetAngle, double currentAngle);
    /***********************************************************
    函数名称：yatDeltaDistance
    函数功能：计算直线起始点到终止点的距离
    入口参数：endPoint:终止点 startPoint:起始点
    出口参数：无
    返回值: 返回值为直线起始点到终止点的距离值
    备 注：
    ***********************************************************/
    double yatDeltaDistance(YATPointXY endPoint,YATPointXY startPoint);
    /***********************************************************
    函数名称：yatDistanceToLine
    函数功能：计算机器人当前位姿到直线的垂直距离
    入口参数：endPoint:直线终止点 startPoint:直线起始点 robotPose:机器人当前位姿
    出口参数：无
    返回值: 返回值为机器人当前位姿到直线的垂直距离
    备 注：
    ***********************************************************/
    double yatDistanceToLine(YATPointXY endPoint,YATPointXY startPoint,YATPointXYZ robotPose);
    /***********************************************************
    函数名称：calParallelLine
    函数功能：根据距离阈值计算接收到的直线的平行线
    入口参数：endPoint:接收的直线终止点 startPoint:接收的直线起始点 d:到直线的距离阈值
    出口参数：无
    返回值: 返回值为包含平行线的终止点和起始点的结构体
    备 注：
    ***********************************************************/
    YATLine yatCalParallelLine(YATPointXY endPoint,YATPointXY startPoint,double d);
    /***********************************************************
    函数名称：yatSign
    函数功能：获取数据的正负
    入口参数：数据值
    出口参数：无
    返回值: 数据为正数时,返回1;负数时返回-1;零时返回0;
    备 注：
    ***********************************************************/
    int yatSign(double data);
    /***********************************************************
    函数名称：yatPoseCallback
    函数功能：获取里程计相关数据,包括里程计位姿以及线速度和角速度
    入口参数：里程计消息指针
    出口参数：无
    备 注：
    ***********************************************************/
    void yatPoseCallback(const nav_msgs::OdometryConstPtr& pose_msg);
    /***********************************************************
    函数名称：yatPlanPointsCallback
    函数功能：获取路径点的坐标信息x和y
    入口参数：路径点消息指针
    出口参数：无
    备 注：
    ***********************************************************/
    void yatPlanPointsCallback(const geometry_msgs::PoseArrayConstPtr& planpoints_msg);
    /***********************************************************
    函数名称：yatAlongEdgeMotion
    函数功能：控制小车沿"边"行走,即控制小车沿直线行走
    入口参数：nextPoint:路径直线的起点,lastPoint:路径直线的终点
    出口参数：无
    返回值: 若小车当前位姿距路径直线的终点小于一定阈值,返回true;否则返回false
    备 注：
    ***********************************************************/
    bool yatAlongEdgeMotion(YATPointXY nextPoint,YATPointXY lastPoint);
    /***********************************************************
    函数名称：yatMotion
    函数功能：实现小车的控制状态之间的转换
    入口参数：无
    出口参数：无
    备 注：
    ***********************************************************/
    bool yatMotion();
    /***********************************************************
    函数名称：yatExecute
    函数功能：执行小车的整体控制程序,实现小车的运动控制
    入口参数：无
    出口参数：无
    备 注：
    ***********************************************************/
    void yatExecute();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber plan_point_sub_;

    geometry_msgs::Twist cmd_vel_msg_;

    //是否更新路径点信息//
    bool yatIsUpdatePLan_;
    
    //接收到的原始直线信息//
    YATLine origline_;
    //转换到里程计坐标系后的直线信息//
    YATLine transline_;
    //直线的平行线信息//
    YATLine parallelline_;
    //机器人位姿
    YATPointXYZ robotpose_;
    //里程计数据
    YATOdomData odomdata_;
};
#endif