#include "mbot_bringup/WeightedFit.h"

CV_IMPLEMENT_QSORT(IntQSort, int, CMP_PTS)  // 该宏利用声明并定义函数IntQSort用于快速排序
int W[MAX_FITPOINTS_CNT];// =(int * )malloc(sizeof(int) * Cnt);// 权值系数	
int yatWeightedFit(int X[], int Y[], int Cnt, YatLinePara *EstLinePara)
{
    // 加权最小二乘法
    // Cnt: 数据点计数
    // EstLinePara : 直线拟合的估计值，可以利用最小二乘法计算得到
    // 利用最小二乘进行估计
    int * Tmp;
    int FlagFlip = 0;// 是否对X,Y进行翻转过
    //FitPara(X , Y , Cnt , EstLinePara , NULL);
    //if(abs(EstLinePara->a) > 1 || EstLinePara->a == NAN || EstLinePara->a == -NAN)
    if( abs(X[0] - X[Cnt - 1]) < abs(Y[0] - Y[Cnt - 1]) )
    {
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        Tmp = X;
        X = Y;
        Y = Tmp;
        FlagFlip = 1;  // 翻转
    }
    
    int i = 0;
    if(W == NULL)
        return -1;
    // 迭代20次
    for(i = 0 ; i < 20 ; i++)
    {
        // 计算权值
        yatCalW(X,Y,Cnt,EstLinePara,W);
        yatFitPara(X,Y,Cnt,EstLinePara,W);// 根据权值拟合参数
    }
    //free(W);
   // EstLinePara->Dis = abs(EstLinePara->b)/(sqrt(EstLinePara->a * EstLinePara->a + EstLinePara->b * EstLinePara->b));
    if(FlagFlip == 0)
    {
        // 未翻转
        EstLinePara->Rho = atan2(EstLinePara->a,1);
    }
    else
    {
        // 翻转过
        if(abs(EstLinePara->a) < 0.00001)
        {
            EstLinePara->a = 100000;
        }	     
        else
        {
            EstLinePara->a =1.0/ EstLinePara->a;
        }	 
        EstLinePara->b = - EstLinePara->b * (EstLinePara->a);
        EstLinePara->Rho = atan2(EstLinePara->a,1);
    }

    //X Y若翻转过，再翻转回去
    if(FlagFlip == 1)
    {
        // 该段直线为斜率大于1
        // 沿45度线进行翻转
        // 将 X 和 Y 进行翻转
        Tmp = X;
        X = Y;
        Y = Tmp;
    }
    //计算线段的两个端点
   if (abs(EstLinePara->a) >= 30000)
   {
       EstLinePara->startPoint.y = Y[0];
       EstLinePara->startPoint.x = (Y[0] - EstLinePara->b)/EstLinePara->a;
        
       EstLinePara->endPoint.y = Y[Cnt-1];
       EstLinePara->endPoint.x = (Y[Cnt-1] - EstLinePara->b)/EstLinePara->a;
   }
   else
   {
       EstLinePara->startPoint.x = X[0];
       EstLinePara->startPoint.y = EstLinePara->a* X[0] + EstLinePara->b;

       EstLinePara->endPoint.x = X[Cnt-1];
       EstLinePara->endPoint.y = EstLinePara->a* X[Cnt-1] + EstLinePara->b;
   }

    //double d;
    //for (int i = 0 ; i < Cnt; i++)
    //{
    //    if (EstLinePara->a >= 30000)
    //    {
    //        d = abs( EstLinePara->a * X[i] - Y[i] + EstLinePara->b)/sqrt(EstLinePara->a*EstLinePara->a + 1);
    //        if (1/*d < 300*/)
    //        {
    //            EstLinePara->startPoint.x = X[i];
    //            EstLinePara->startPoint.y = EstLinePara->a*X[i] + EstLinePara->b;
    //            break;
    //        }
    //    }

    //   
    //}

    //for (int i = Cnt-1 ; i >=0; i--)
    //{
    //    d = abs( EstLinePara->a * X[i] - Y[i] + EstLinePara->b)/sqrt(EstLinePara->a*EstLinePara->a + 1);
    //    if (1/*d < 300*/)
    //    {
    //        EstLinePara->endPoint.x = X[i];
    //        EstLinePara->endPoint.y = EstLinePara->a*X[i] + EstLinePara->b;
    //        break;
    //    }
    //}
    return 0;
}

int yatCmp(const void *a, const void *b)
{
    return ((*(int*)a-*(int*)b>0)?1:-1);
}

int yatMed(int R[] , int Cnt)// 求取中值
{
    // qsort(R , Cnt , sizeof(R[0]),yatCmp);
    IntQSort(R , Cnt , 0);
    return R[Cnt/2];
}
int yatCalW(int X[] , int Y[] , int Cnt , YatLinePara * EstLinePara , int W[] )
{
    int i = 0;
    double a = (double)EstLinePara->a;
    double b = (double)EstLinePara->b;
    
    int Median = 0;
    double u;
    double tmp;
    for(i = 0; i < Cnt ; i++)
    {
        tmp = (int)abs(Y[i] - a * X[i] - b );
        W[i]=tmp;

        // std::cout<<"W[i]= "<<W[i]<<std::endl; //w[i]==0
    }
    
    Median = yatMed(W , Cnt);
    Median = Median > 2 ? Median : 2;

    //std::cout<<"median= "<<Median<<std::endl; //median ==2 

    for(i = 0 ; i < Cnt ; i++)
    {
        u =(double)( W[i]/(COEFFICIENT * Median) );
        // u =(double)( W[i]/(5.0 * Median) );

        if(u < 1)
        {
            W[i] =(int)((1 - u * u) * (1 - u * u) * 100);   //将W范围限制在0-100
            //W[i] = (int)((1-u)*(1-u)*100);
            // std::cout<<"W[i]==== "<<W[i]<<std::endl;
        }
        else{
            W[i] = 0;
        }

        // W[i]=1;
    }

    return 0;
}
int yatFitPara(int X[], int Y[], int Cnt, YatLinePara * EstLinePara, int W[])
{
    int i = 0;
    long long P1 = 0; // sum(wi*xi*yi);
    long long P2 = 0; // sum(wi * xi * xi)
    long long P3 = 0; // sum(wi * xi)
    long long P4 = 0; // sum(wi * yi)
    long long P5 = 0; // sum(wi)
    // 直接进行最小二乘拟合，即所有数据的权值相等//
    if(W == NULL) 
    {
        //
        for( i = 0 ; i < Cnt ;i++)
        {
            P1 +=  X[i] * Y[i];
            P2 +=   X[i] * X[i];
            P3 +=  X[i];
            P4 +=   Y[i];
            P5 += 1;
        }
    }
    //加权最小二乘拟合//
    else
    {   
        for( i = 0 ; i < Cnt ;i++)
        {
            P1 += W[i] * X[i] * Y[i];
            P2 += W[i] * X[i] * X[i];
            P3 += W[i] * X[i];
            P4 += W[i] * Y[i];
            P5 += W[i];
        }
    }

    EstLinePara->a = ( ((double)( ((double)P1) * ((double)P5) - P4 * P3)) / ( (double)(((double)P2) * ((double)P5) - P3*P3) ) );
    EstLinePara->b = (P1 - P2 * EstLinePara->a)/P3;
    return 0; 
}
