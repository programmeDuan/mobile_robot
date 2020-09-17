#ifndef WEIGHTED_FIT_H
#define WEIGHTED_FIT_H
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "mbot_bringup/Qsort.h"
#define MAX_FITPOINTS_CNT 1000
//加权拟合中的系数//
#define COEFFICIENT 5
//#include "ImgProcess.h"

typedef struct
{
    int x;
    int y;
}YatPoint;
typedef struct
{
    double a;//y = a*x + b
    double b;
    double Rho; // 该段直线的倾角
    YatPoint startPoint;
    YatPoint endPoint;
}YatLinePara;

int yatCmp(const void *a, const void *b);
int yatMed(int R[], int Cnt);// 求取中值
int yatCalW(int X[], int Y[], int Cnt, YatLinePara * EstLinePara, int W[]);
int yatFitPara(int X[], int Y[], int Cnt,YatLinePara * EstLinePara, int W[]);
// int CalW(float X[] , float Y[] , int Cnt , LinePara * EstLinePara , int W[]);
// int FitPara(float X[] , float Y[] , int Cnt ,LinePara * EstLinePara , int W[]);
int yatWeightedFit(int X[], int Y[], int Cnt, YatLinePara * EstLinePara);
#define CMP_PTS(x, y)   (x < y)    //  用于快速排序比较x < y , 得到的结果为升序排列
#endif


