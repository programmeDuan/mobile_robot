/***********************************************************
文件名称：laser_recharge
作 者： 段广学
版 本：　０.0.3
说 明：利用激光雷达实现扫雪机的自动回充功能
修改记录：2020-06-12
***********************************************************/
//ros//
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
//c++//
#include <vector>
#include <iostream>
//opencv//
// #include "opencv2/opencv.hpp"
// #include "opencv2/highgui/highgui.hpp"
//Eigen//
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "mbot_bringup/WeightedFit.h"

using namespace std;
// using namespace cv;

#define PI 3.1415926
//提取出的雷达图像尺寸//
#define RADAR_IMAGE_WDITH 720
#define RADAR_IMAGE_HEIGHT 720
//提取出的直线上点的数目的范围//
#define MIN_POINT_NUM 15
#define MAX_POINT_NUM 150
//激光雷达有效数据的最小最大距离范围//
// #define MIN_VALID_DIS 700
#define MIN_VALID_DIS 1250
#define MAX_VALID_DIS 2500
//雷达数据的比例系数//
#define K 1000
//判断向量正交的角度范围//
#define MIN_ANGLE 80*PI/180
#define MAX_ANGLE 100*PI/180
//定义充电桩模型的长度//
#define MODEL_MIN_LENGTH 900 
#define MODEL_MAX_LENGTH 1030
//测试使用的 
// #define MODEL_MIN_LENGTH 650   
// #define MODEL_MAX_LENGTH 700
//扫雪机长度//
#define SNOWBLOWER_LENGTH 680
//直线斜率值//
#define SLOPE 100000

//常用颜色//
/*Colour      Red      Green      Blue      值     
    白色   White    255    255    255    16777215     
    红色    Red    255    0    0    255     
    深红色    Dark    Red    128    0    0    128     
    绿色    Green    0    255    0    65280     
    深绿色    Dark    Green    0    128    0    32768     
    蓝色    Blue    0    0    255    16711680       
    紫红色    Magenta    255    0    255    16711935            
    深紫    Dark    Cyan    0    128    128    8421376     
    黄色    Yellow    255    255    0    65535     
    棕色    Brown    128    128    0    32896  
 */
//<10种常用的颜色//
static int usualColor[15] = {16777215,255,128,65280,32768,
                             16711680,16711935,8421376,65535,32896 }; 
//YATPointXY 点结构体//
typedef struct
{
    double x;
    double y;
}YATPointXY;

typedef struct
{
    double x;
    double y;
    double z;

}YATPointXYZ;

//YATLine 直线结构体//
typedef struct
{
    YATPointXY startpoint;
    YATPointXY endpoint;
}YATLine;

class YATLaserRecharge
{
public:
    /***********************************************************
    函数名称：YATLaserRecharge
    函数功能：构造函数,初始化部分参数,并创建需要订阅和发布的话题
    入口参数：
    出口参数：
    备 注:
    ***********************************************************/
    YATLaserRecharge();
    ~YATLaserRecharge();
    /***********************************************************
    函数名称：yatLaserScanCallback
    函数功能：对激光数据进行预处理,得到待分割的激光数据
    入口参数：激光消息的指针
    出口参数：待分割的激光数据
    备 注：
    ***********************************************************/
    void yatLaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan_msg);
    /***********************************************************
    函数名称：yatOdomCallback
    函数功能：得到扫雪机左右轮的里程数据
    入口参数：里程信息的指针
    出口参数：左右轮以及中心里程数据
    备 注：
    ***********************************************************/
    void yatOdomCallback(const geometry_msgs::TwistConstPtr& odom_msg);
    /***********************************************************
    函数名称：yatGoHomeCallback
    函数功能：回充回调函数,接收到消息执行回充
    入口参数：回充消息的指针
    出口参数：
    备 注：
    ***********************************************************/
    void yatGoHomeCallback(const std_msgs::BoolConstPtr& gohome_msg);
    /***********************************************************
    函数名称：yatStartWorkCallback
    函数功能：开始工作回调函数,接收到消息机器开始工作
    入口参数：开始工作消息的指针
    出口参数：
    备 注：
    ***********************************************************/
    void yatStartWorkCallback(const std_msgs::BoolConstPtr& startwork_msg);
    /***********************************************************
    函数名称：yatRegionSegmentation
    函数功能：对激光雷达的扫描区域进行分割
    入口参数：
    出口参数：
    返回值: 分割后的区域数
    备 注：
    ***********************************************************/
    int yatRegionSegmentation();
    /***********************************************************
    函数名称：yatPolyContourFit
    函数功能：进行多边形拟合
    入口参数: X和Y:轮廓上的点的坐标 n:轮廓点数目  Eps:拟合精度
    出口参数：
    返回值:  若该轮廓段需要分段，则返回分段点在该轮廓点列中的索引，否则，返回 0 表示不需要分段
    备 注：
    ***********************************************************/
    int yatPolyContourFit(int* X, int* Y, int n, int Eps);
    /***********************************************************
    函数名称：yatBreakPolyLine
    函数功能：进行折线拆分
    入口参数: 
    出口参数：
    返回值:  拆分完成后的直线数
    备 注：
    ***********************************************************/
    int yatBreakPolyLine();
    /***********************************************************
    函数名称：yatFitLine
    函数功能：利用最小二乘法进行直线拟合
    入口参数: scanData:激光雷达距离数据 scanTheta:激光雷达的角度数据
    出口参数：拟合得到的直线
    备 注：
    ***********************************************************/
    void yatFitLine(vector<YatLinePara>& FittedLine,vector<int>& scanData,vector<double>& scanTheta);
    /***********************************************************
    函数名称：yatDrawRadarLine
    函数功能：绘制并显示拟合直线
    入口参数: FittedLine:拟合直线信息
    出口参数：laserImage:存储拟合直线的图像
    备 注：
    ***********************************************************/
    // void yatDrawRadarLine(vector<YatLinePara>& FittedLine,IplImage* laserImage);
    // void yatDrawRadarLine(vector<YatLinePara>& FittedLine,cv::Mat laserImage);
    /***********************************************************
    函数名称：yatCreateRadarImage
    函数功能：显示分割的laser数据
    入口参数: RadarRho:分割后的激光雷达距离数据 RadarTheta:激光雷达的角度数据
    出口参数：RadarImage: 存储分割后的雷达数据的图像
    备 注：
    ***********************************************************/
    // void yatCreateRadarImage(IplImage* RadarImage,vector<int>& RadarRho,vector<double>& RadarTheta);
    // void yatCreateRadarImage(cv::Mat laserImage,vector<int>& RadarRho,vector<double>& RadarTheta);
     /***********************************************************
    函数名称：yatChargingStandIdentification
    函数功能：将拟合的直线与充电桩的特征模型进行匹配识别
    入口参数: FittedLine:拟合的直线信息
    出口参数：
    备 注：
    ***********************************************************/
    void yatChargingStandIdentification(vector<YatLinePara>& FittedLine);
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
    double yatDistanceToLine(YATPointXY endPoint,YATPointXY startPoint);
     /***********************************************************
    函数名称：yatSign
    函数功能：获取数据的正负
    入口参数：数据值
    出口参数：无
    返回值: 数据为正数时,返回1;负数时返回-1;零时返回0;
    备 注：
    ***********************************************************/
    int yatSign(double data);
    int yatCalMoveDistance(double cur_mileage);
     /***********************************************************
    函数名称：yatAlongEdgeMotion
    函数功能：控制小车沿"边"行走,即控制小车沿直线行走
    入口参数：nextPoint:路径直线的起点,lastPoint:路径直线的终点
    出口参数：无
    返回值: 若小车当前位姿距路径直线的终点小于一定阈值,返回true;否则返回false
    备 注：2020-6-12:调试要发布出去的小车角速度和线速度,
    ***********************************************************/
    bool yatBackMotion(YATPointXY nextPoint,YATPointXY lastPoint);
    bool yatForwardMotion(YATPointXY nextPoint,YATPointXY lastPoint);
    /***********************************************************
    函数名称：yatGoHomeMotion
    函数功能：实现小车入库的控制状态之间的转换
    入口参数：无
    出口参数：无
    返回值:若小车沿直线行走时到达直线的终止点,返回true,否则false
    备 注：
    ***********************************************************/
    bool yatGoHomeMotion();
    /***********************************************************
    函数名称：yatStartWorkMotion
    函数功能：实现小车出库开始工作的控制状态之间的转换
    入口参数：无
    出口参数：无
    返回值:若小车沿直线行走时到达直线的终止点,返回true,否则false
    备 注：
    ***********************************************************/
    bool yatStartWorkMotion();
    /***********************************************************
    函数名称：yatExecute
    函数功能：运行回充模块程序,识别充电座进行自动回充
    入口参数: 
    出口参数：
    备 注：
    ***********************************************************/
    void yatExecute();

    std::vector<int> use_scan_data_;//有效数据
    std::vector<double> use_angle_data_;//有效数据对应的角度

    std::vector<int> seg_scan_data_;//分割的数据
    std::vector<double> seg_angle_data_;
    // std::vector<int> seg_index;    //分割点对应的index 可以求出每条线段的数据个数;

    std::vector<int> break_scan_data_;//拆分的数据
    std::vector<double> break_angle_data_;
    std::vector<YatLinePara> FittedLine;

private:
    //创建ros句柄//
    ros::NodeHandle nh_;
    //订阅和发布话题//
    ros::Subscriber laserscan_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_; 
    ros::Subscriber gohome_sub_;
    ros::Subscriber startwork_sub_;
    //创建速度消息//
    geometry_msgs::Twist cmd_vel_msg_;
    //雷达的角度分辨率和最小角度//
    float angle_increment_;
    double min_angle_;
    //里程计数据:左里程和右里程以及中心里程//
    double left_mileage_;
    double right_mileage_;
    double center_mileage_;
    //判断两向量的是否正交的角度//
    double orthogonal_angle_;
    //扫雪机到充电桩距离//
    int dist_to_stand_;
    //充电桩位置//
    // cv::Point2i stand_pos_;
    YatPoint stand_pos_;
    //扫雪机在雷达坐标系下位姿//
    // cv::Point3f robot_pose_;
    YATPointXYZ robot_pose_;
    //后退行走的直线//
    YATLine along_line_;
    //前进行走的直线//
    // YATLine 
    //是否识别到充电桩//
    bool finding_stand_;
    //是否完成沿直线行走//
    bool finished_along_edge_;
    //是否回充电桩//
    bool go_home_;
    //是否开始工作//
    bool start_work_;
};
