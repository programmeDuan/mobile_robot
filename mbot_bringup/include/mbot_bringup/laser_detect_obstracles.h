/***********************************************************
文件名称：laser_detect_obstracles
作 者： 段广学
版 本：　０.0.1
说 明：利用激光雷达的完成障碍物检测功能
修改记录：2020-6-20
***********************************************************/
//ros//
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
//c++//
#include <iostream>

#include <vector>
#include <cmath>
// #include <ctime>
// #include <sys/time.h>

//常量//
#define PI 3.1415926
#define ANGLE_THRESHOLD 15*PI/180
//雷达到障碍物的最小安全距离//
#define DISTANCE 1
//超声检测障碍物的安全距离//
#define SONAR_DISTANCE 0.7


class YATDetectObstracles
{
public:
    YATDetectObstracles();
    ~YATDetectObstracles();
    /***********************************************************
    函数名称：yatScanCallback
    函数功能：利用激光数据实现障碍物检测
    入口参数：激光消息的指针
    出口参数：
    备 注：
    ***********************************************************/
    void yatScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
    /***********************************************************
    函数名称：yatSonarCallback
    函数功能：利用超声数据实现障碍物检测
    入口参数：超声消息的指针
    出口参数：
    备 注：
    ***********************************************************/
   void yatSonarCallback(const geometry_msgs::PoseStampedPtr& sonar_msg);
    /***********************************************************
    函数名称：yatPublishZeroVelocity
    函数功能：发布为0的速度给扫雪机让其停下来
    入口参数：激光消息的指针
    出口参数：
    备 注：
    ***********************************************************/
    void yatPublishZeroVelocity();
private:
    //创建ros句柄//
    ros::NodeHandle nh_;
    //订阅和发布话题//
    ros::Subscriber laserscan_sub_;
    ros::Subscriber sonar_sub_;
    ros::Publisher laserdetect_pub_;
    ros::Publisher sonardetect_pub_;
    ros::Publisher cmd_pub_; 
};