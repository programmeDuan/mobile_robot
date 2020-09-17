/***********************************************************
文件名称：get_lane_line
作 者： 王斌
版 本：　０.0.1
说 明：从ＲＧＢ图像中提取车道线
修改记录：
***********************************************************/
#ifndef RGB_LIN_HPP
#define RGB_LIN_HPP

#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include "ranker.h"
#include <iostream>

#define DEBUG_FLAG 1
#define FILENAME "/home/wangbin/caltech-lane-detection/caltech-lanes/cordova1/f00009.png"
#define IMG_WIDTH 640
#define IMG_HEIGHT 480


//Line 结构体//
struct YatLine
{
  //起始点//
  cv::Point2f startPoint;
  //结束点//
  cv::Point2f endPoint;
  //直线的置信度//
  float score;
};

class YatProcRgbLine
{
public:
    ~YatProcRgbLine();

    /***********************************************************
    函数名称：yatGetLines
    函数功能：获取图像上的直线
    入口参数：无
    出口参数：inlines:提取到的直线
    备 注：
    ***********************************************************/
    int yatGetLines(cv::Mat ipmImage, std::vector<YatLine> &inlines);

    /***********************************************************
    函数名称：yatSetParam
    函数功能：参数初始化
    入口参数：无
    出口参数：无
    备 注：
    ***********************************************************/
    void yatSetParam();

private:

    cv::Mat mapx, mapy; //x,y 坐标的映射关系

    /***********************************************************
    函数名称：yatPerspectiveToMaps
    函数功能：获取原图与转换后的图的映射关系，
    入口参数：perspective_mat：透视变换矩阵
            width，height:透视变换原始图像的宽和高
    出口参数：mapx:存放x坐标的映射
            mapy:存放y坐标的映射
    备 注：
    ***********************************************************/
    void yatPerspectiveToMaps(cv::Mat perspective_mat, int width , int height, cv::Mat &mapsx, cv::Mat &mapsy);

    /***********************************************************
    函数名称：yatGetMap
    函数功能：获取原图与转换后的图的映射关系，
    入口参数：无
    出口参数：mapx:存放x坐标的映射
            mapy:存放y坐标的映射
    返回：   成功返回０，错误返回－１
    备 注：
    ***********************************************************/
    int yatGetMap(cv::Mat &mapx, cv::Mat &mapy);
    
    /***********************************************************
    函数名称：yatFilterLines
    函数功能：卷积核滤波处理图像
    入口参数：inImage: 输入的图像
    出口参数：outImage： 输出滤波后的图像
    备 注：
    ***********************************************************/
    void yatFilterLines(cv::Mat inImage, cv::Mat &outImage);

    /***********************************************************
    函数名称：yatGetQuantile
    函数功能：获取前qtile比例的值
    入口参数：mat: 输入的图像
            qtile:比例值
    出口参数：输出的值
    备 注：
    ***********************************************************/
    float yatGetQuantile(const cv::Mat mat, float qtile);

    /***********************************************************
    函数名称：yatGetMatLocalMax
    函数功能：用r和theta获取线段在直线上的端点
    入口参数：r:r
            theta:theta
            bbox:端点的区域
    出口参数：outlines:输出的线段端点
    备 注：
    ***********************************************************/
    void yatGetMatLocalMax(const cv::Mat inMat, vector<double> &localMaxima, vector<cv::Point> &localMaximaLoc, double threshold);

    /***********************************************************
    函数名称：yatIsPointInside
    函数功能：用r和theta获取线段在直线上的端点
    入口参数：r:r
            theta:theta
            bbox:端点的区域
    出口参数：outlines:输出的线段端点
    备 注：
    ***********************************************************/
    bool yatIsPointInside(cv::Point2f point, CvSize bbox);

    /***********************************************************
    函数名称：yatIntersectLineRThetaWithBB
    函数功能：用r和theta获取线段在直线上的端点
    入口参数：r:r
            theta:theta
            bbox:端点的区域
    出口参数：outlines:输出的线段端点
    备 注：
    ***********************************************************/
    void yatIntersectLineRThetaWithBB(float r, float theta, const cv::Size bbox, YatLine *outLine);

    /***********************************************************
    函数名称：yatGetHoughTransformLines
    函数功能：用hough变换获取图像上的直线
    入口参数：inImage
    出口参数：inlines:提取到的直线
            lineScores:对应直线的分数
    备 注：
    ***********************************************************/
    void yatGetHoughTransformLines(cv::Mat inImage, vector <YatLine> *lines, vector <float> *lineScores);
};
#endif



