#include "mbot_bringup/laser_recharge.h"

YATLaserRecharge::YATLaserRecharge():angle_increment_(0),min_angle_(0),left_mileage_(0),right_mileage_(0),center_mileage_(0),orthogonal_angle_(0.0),dist_to_stand_(0)
{
    laserscan_sub_ = nh_.subscribe("/scan",10,&YATLaserRecharge::yatLaserScanCallback,this);
    odom_sub_ = nh_.subscribe("/Odom",10,&YATLaserRecharge::yatOdomCallback,this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    gohome_sub_ = nh_.subscribe("go_home",10,&YATLaserRecharge::yatGoHomeCallback,this);
    startwork_sub_ = nh_.subscribe("start_work",10,&YATLaserRecharge::yatStartWorkCallback,this);
    // robot_pose_ = cv::Point3f(0,0,0);
    finding_stand_ = false;
    finished_along_edge_ = false;
    go_home_ =false;
    start_work_ = false;
}

YATLaserRecharge::~YATLaserRecharge()
{

}
/***********************************************************
    函数名称：yatLaserScanCallback
    函数功能：对激光数据进行预处理,得到待分割的激光数据
    入口参数：激光消息的指针
    出口参数：待分割的激光数据
    备 注：
***********************************************************/
void YATLaserRecharge::yatLaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan_msg)
{
    int scan_data = 0;
    double theta = 0.0;
    use_scan_data_.clear();
    use_angle_data_.clear();
    angle_increment_ = laser_scan_msg->angle_increment;
    min_angle_ = laser_scan_msg->angle_min;
    // std::cout<<laser_scan_msg->ranges.size()<<std::endl;

    for (int i = 0; i < laser_scan_msg->ranges.size(); i++)
    {
        scan_data = K * laser_scan_msg->ranges.at(i);
        theta = laser_scan_msg->angle_min + laser_scan_msg->angle_increment * i;
        if((MIN_VALID_DIS <= scan_data) && (scan_data <= MAX_VALID_DIS))
        {
            use_scan_data_.push_back(scan_data);
            use_angle_data_.push_back(theta);
        }
    }
}
/***********************************************************
    函数名称：yatOdomCallback
    函数功能：得到扫雪机左右轮的里程数据
    入口参数：里程信息的指针
    出口参数：左右轮以及中心里程数据
    备 注：
***********************************************************/
void YATLaserRecharge::yatOdomCallback(const geometry_msgs::TwistConstPtr& odom_msg)
{
    left_mileage_ = odom_msg->angular.x;
    right_mileage_ = odom_msg->angular.y;
    center_mileage_ = 0.5 * (left_mileage_+right_mileage_);
    // yatCalMoveDistance(center_mileage_);
}
/***********************************************************
    函数名称：yatGoHomeCallback
    函数功能：回充回调函数,接收到消息执行回充
    入口参数：回充消息的指针
    出口参数：
    备 注：
 ***********************************************************/
void YATLaserRecharge::yatGoHomeCallback(const std_msgs::BoolConstPtr& gohome_msg)
{
    if (gohome_msg->data)
    {
        go_home_ = true;
    }
}
/***********************************************************
    函数名称：yatStartWorkCallback
    函数功能：开始工作回调函数,接收到消息机器开始工作
    入口参数：开始工作消息的指针
    出口参数：
    备 注：
***********************************************************/
void YATLaserRecharge::yatStartWorkCallback(const std_msgs::BoolConstPtr& startwork_msg)
{
    if(startwork_msg->data)
    {
        start_work_ = true;
    }
}
/***********************************************************
    函数名称：yatRegionSegmentation
    函数功能：对激光雷达的扫描区域进行分割
    入口参数：
    出口参数：
    返回值: 分割后的区域数
    备 注：
***********************************************************/
int YATLaserRecharge::yatRegionSegmentation()
{
    int segCnt = 0;
    int scan_data = 0;
    int lastscan_data = 0;
    double theta = 0;
    int dist_threshold = 0;

    seg_scan_data_.clear();
    seg_angle_data_.clear();

    // seg_scan_data_.push_back(lastscan_data);
    // seg_angle_data_.push_back(theta);

    //区域分割得到分割点//
    for(int i=0;i<use_scan_data_.size();i++)
    {
        scan_data = use_scan_data_.at(i);
        theta = use_angle_data_.at(i);

        seg_scan_data_.push_back(scan_data);
        seg_angle_data_.push_back(theta);
        //与阈值比较判断是否为分割点//
        dist_threshold =use_scan_data_.at(i)*sin(angle_increment_)/sin(10*PI/180-angle_increment_)+15;
        // cout<<"dist_threshold="<<dist_threshold<<endl;
        if(i<((int)use_scan_data_.size()-1))
        {
            if(fabs(use_scan_data_.at(i+1)-use_scan_data_.at(i)) > dist_threshold)
            {
                //设置分割点的数据//
                seg_scan_data_.push_back(-1);
                seg_angle_data_.push_back(1000.0);
                segCnt++;
            }
        }

        else if(i==((int)use_scan_data_.size()-1))
        {
            if(fabs(use_scan_data_.at(i)-use_scan_data_.at(0)) > dist_threshold)
            {
                seg_scan_data_.push_back(-1);
                seg_angle_data_.push_back(1000.0);
                segCnt++;
            }
        }

        else
        {
            continue;
        }

        // lastscan_data =scan_data;
    }

    seg_scan_data_.push_back(-1);
    seg_angle_data_.push_back(1000.0);

    // std::cout<<"segCnt:"<<segCnt<<std::endl;
    return segCnt;
}
/***********************************************************
    函数名称：yatPolyContourFit
    函数功能：进行多边形拟合
    入口参数: X和Y:轮廓上的点的坐标 n:轮廓点数目  Eps:拟合精度
    出口参数：
    返回值:  若该轮廓段需要分段，则返回分段点在该轮廓点列中的索引，否则，返回 0 表示不需要分段
    备 注：
***********************************************************/
int YATLaserRecharge::yatPolyContourFit(int* X, int* Y, int n, int Eps)
{
    double dis = sqrt((double)(((X[0] - X[n - 1])*(X[0] - X[n - 1])) +  
                     ((Y[0] - Y[n - 1])* (Y[0] - Y[n - 1]))));
    double cosTheta = (X[n- 1] - X[0]) / dis;
    double sinTheta = - ( Y[n- 1] - Y[0] )/dis;
    double MaxDis = 0;
    int i ;
    int MaxDisInd = -1;
    double dbDis;
    for(i = 1 ; i < n - 1 ; i++)
    {
        // 进行坐标旋转，求旋转后的点到x轴的距离//
        dbDis = abs( (Y[i] - Y[0]) * cosTheta + (X[i] - X[0])* sinTheta);
        if( dbDis > MaxDis)
        {
            MaxDis = dbDis;
            MaxDisInd = i;
        }
    }
    if(MaxDis > Eps)
    {
        return MaxDisInd;
        //        cout << "Line 1 : " << endl;
        //        cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y << endl;
        //        cout << "angle： "<<180 * atan2(Points[0].y - Points[MaxDisInd].y , Points[0].x - Points[MaxDisInd].x ) / 3.1415926;
        //        cout << "Line 2 :" << endl;
        //        cout << "Start :" << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
        //        cout << "angle： "<< 180 * atan2(Points[n - 1].y - Points[MaxDisInd].y , Points[n - 1].x - Points[MaxDisInd].x ) / 3.1415926;
    }
    //    else{
    //        cout << "Line 1 : " << endl;
    //        cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
    //        cout << "angle： "<<180 * atan2(Points[n - 1].y - Points[0].y , Points[n - 1].x - Points[0].x ) / 3.1415926;

    //    }
    return 0;
}
/***********************************************************
    函数名称：yatBreakPolyLine
    函数功能：进行折线拆分
    入口参数: 
    出口参数：
    返回值:  拆分完成后的直线数
    备 注：
***********************************************************/
int YATLaserRecharge::yatBreakPolyLine()
{
    int scan_data =0;
    double theta = 0;
    int X[1200]={0};
    int Y[1200]={0};
    int scan_dataCopy[1200]={0};
    double thetaCopy[1200]={0};
    int pointCnt=0;
    int lineCnt =0;
    int N =0;

    break_scan_data_.clear();
    break_angle_data_.clear();

    for(int i=0;i<seg_scan_data_.size();i++)
    {
        scan_data=seg_scan_data_.at(i);
        theta=seg_angle_data_.at(i);

        if(scan_data<0)
        {
            if(MIN_POINT_NUM < pointCnt)
            {
                N=yatPolyContourFit(X,Y,pointCnt,50);
                // std::cout<<"N:"<<N<<std::endl;
                //std::cout<<"pointCnt: "<<pointCnt<<std::endl;
                //如果不是直线，需要添加新的断点//
                if(N==0) //轮廓段不需要分段
                {
                    lineCnt++;
                    for(int j=0;j<pointCnt;j++)
                    {
                        break_scan_data_.push_back(scan_dataCopy[j]);
                        break_angle_data_.push_back(thetaCopy[j]);
                    }
                    break_scan_data_.push_back(-1);
                    break_angle_data_.push_back(1000.0);
                }
                else if(N>0)
                {
                    lineCnt+=2;

                    for(int j=0;j<N;j++)
                    {
                        break_scan_data_.push_back(scan_dataCopy[j]);
                        break_angle_data_.push_back(thetaCopy[j]);
                    }
                    break_scan_data_.push_back(-1);
                    break_angle_data_.push_back(1000.0);

                    for(int j=N;j<pointCnt;j++)
                    {
                        break_scan_data_.push_back(scan_dataCopy[j]);
                        break_angle_data_.push_back(thetaCopy[j]);
                    }
                    break_scan_data_.push_back(-1);
                    break_angle_data_.push_back(1000.0);
                }
            }
            pointCnt=0;
            continue;
        }

        X[pointCnt]=scan_data*cos(theta);
        Y[pointCnt]=scan_data*sin(theta);
        scan_dataCopy[pointCnt]=scan_data;
        thetaCopy[pointCnt]=theta;
        pointCnt++;
    }

    std::cout<<"lineCnt:"<<lineCnt<<std::endl;
    return lineCnt;
}
/***********************************************************
    函数名称：yatFitLine
    函数功能：利用最小二乘法进行直线拟合
    入口参数: scanData:激光雷达距离数据 scanTheta:激光雷达的角度数据
    出口参数：拟合得到的直线
    备 注：
***********************************************************/
void YATLaserRecharge::yatFitLine(vector<YatLinePara>& FittedLine,vector<int>& scanData,vector<double>& scanTheta)
{
    int scan_data =0;
    double theta = 0;
    int X[1200]={0};
    int Y[1200]={0};

    int pointCnt =0;
    YatLinePara tmpLinePara;
    FittedLine.clear();

    for(int i=0;i<scanData.size();i++)
    {
        scan_data=scanData.at(i);
        theta=scanTheta.at(i);
        
        if(scan_data<0)
        {
            yatWeightedFit(X,Y,pointCnt,&tmpLinePara);
            // if(tmpLinePara.a == SLOPE)
            // {
            //     FittedLine.push_back(tmpLinePara);
            //     pointCnt = 0;
            // }
            FittedLine.push_back(tmpLinePara);
            pointCnt=0;
            continue;
        }

        X[pointCnt]=scan_data*cos(theta);
        Y[pointCnt]=scan_data*sin(theta);
        pointCnt++;
    }

    for(int i=0;i<FittedLine.size();i++)
    {
        std::cout<<"a: "<<FittedLine.at(i).a<<"  b: "<<FittedLine.at(i).b<<" ";
        std::cout<<"x1: "<<FittedLine.at(i).startPoint.x<<" "
                 <<"y1: "<<FittedLine.at(i).startPoint.y<<" "
                 <<"x2: "<<FittedLine.at(i).endPoint.x<<" "
                 <<"y2: "<<FittedLine.at(i).endPoint.y<<std::endl;
    }
}
/***********************************************************
    函数名称：yatDrawRadarLine
    函数功能：绘制并显示拟合直线
    入口参数: FittedLine:拟合直线信息
    出口参数：laserImage:存储拟合直线的图像
    备 注：
***********************************************************/
// void YATLaserRecharge::yatDrawRadarLine(vector<YatLinePara>& FittedLine,IplImage* laserImage)
// // void YATLaserRecharge::yatDrawRadarLine(vector<YatLinePara>& FittedLine,cv::Mat laserImage)
// {
//     //在中心加上一个圆心//
//     int dx =  RADAR_IMAGE_WDITH/2;
//     int dy =  RADAR_IMAGE_HEIGHT*3/4;
//     cvCircle(laserImage, cvPoint(dx,dy),3, CV_RGB(0,255,255), -1, 8,0);
//     int x1,y1,x2,y2;
//     //颜色//
//     int colorIndex = 0,colorRGB;
//     int R = 255,G = 0,B = 0;
//     for (int i = 0; i < FittedLine.size();i++)
//     {
//         //雷达数据断点标志//
//         colorRGB = usualColor[colorIndex];
//         R = colorRGB/65536;
//         G = (colorRGB%65536)/256;
//         B = colorRGB%256; 
//         colorIndex = (colorIndex + 1)%10;

//         x1 = FittedLine.at(i).startPoint.x;
//         y1 = FittedLine.at(i).startPoint.y;

//         x2 = FittedLine.at(i).endPoint.x;
//         y2 = FittedLine.at(i).endPoint.y;

//         //转化为Radar图坐标系下//
//         x1 = x1/10  + dx;
//         y1 = -y1/10 + dy;
//         x2 = x2/10  + dx;
//         y2 = -y2/10 + dy;
//         // cout<<"x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<endl;
//         // cvLine(laserImage,cvPoint(x2,y2),cvPoint(x1,y1),CV_RGB(R,G,B),2,8,0);
//         cvLine(laserImage,cvPoint(x1,y1),cvPoint(x2,y2),CV_RGB(R,G,B),2,8,0);
//     }
// }
/***********************************************************
    函数名称：yatCreateRadarImage
    函数功能：显示分割的laser数据
    入口参数: RadarRho:分割后的激光雷达距离数据 RadarTheta:激光雷达的角度数据
    出口参数：RadarImage: 存储分割后的雷达数据的图像
    备 注：
***********************************************************/
//  void YATLaserRecharge::yatCreateRadarImage(IplImage* RadarImage,vector<int>& RadarRho,vector<double>& RadarTheta)
//  {
//  //RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,1);
//     cvZero(RadarImage);
//     //在中心加上一个圆心
//     int dx =  RADAR_IMAGE_WDITH/2;
//     int dy =  RADAR_IMAGE_HEIGHT*3/4;

//     cvCircle(RadarImage, cvPoint(dx,dy),3, CV_RGB(0,255,255), -1, 8,0);

//     int x,y;
//     double theta,rho;
//     unsigned char *pPixel = 0;

//     //颜色
//     int colorIndex = 0,colorRGB;
//     int R = 255,G = 0,B = 0;
//     int pointCnt = 0;
  
//     for (int i = 0; i < RadarRho.size();i++)
//     {
//         //theta = (pointCnt/4.0 - 45)*pi/180;
//         theta = RadarTheta.at(i);
//         rho = RadarRho.at(i);
//         if (rho < 0)
//         {
//            //雷达数据断点标志
//             colorRGB = usualColor[colorIndex];
//             R = colorRGB/65536;
//             G = (colorRGB%65536)/256;
//             B = colorRGB%256; 
//             colorIndex = (colorIndex + 1)%10;
//         }
//         else 
//         {
//             pointCnt++;
//         }

//         x = (int)(rho*cos(theta)/10) + dx;
//         y = (int)(-rho*sin(theta)/10)+ dy;

//         if (x >= 0 && x < RADAR_IMAGE_WDITH && y >= 0 && y < RADAR_IMAGE_HEIGHT)
//         {
//             pPixel = (unsigned char*)RadarImage->imageData + y*RadarImage->widthStep + 3*x;
//             pPixel[0] = B;
//             pPixel[1] = G;
//             pPixel[2] = R;
//         }
//         else
//         {
//             // cout<<"x: "<<x<<"  y: "<<y<<endl;
//         }
//     }
// }
 /***********************************************************
    函数名称：yatChargingStandIdentification
    函数功能：将拟合的直线与充电桩的特征模型进行匹配识别
    入口参数: FittedLine:拟合的直线信息
    出口参数：
    备 注：
***********************************************************/
void YATLaserRecharge::yatChargingStandIdentification(vector<YatLinePara>& FittedLine)
{
    int line_length = 0;//直线长度
    int chargestand2laser_dist = 0;//充电桩到雷达距离
    int startPoint_dis = 0;
    int endPoint_dis = 0;

    for(int i = 0; i < FittedLine.size(); i++)
    {
        if(FittedLine.at(i).a == SLOPE)
        {
            line_length = sqrt((FittedLine.at(i).endPoint.x-FittedLine.at(i).startPoint.x)*(FittedLine.at(i).endPoint.x-FittedLine.at(i).startPoint.x)+
                           (FittedLine.at(i).endPoint.y-FittedLine.at(i).startPoint.y)*(FittedLine.at(i).endPoint.y-FittedLine.at(i).startPoint.y));
            std::cout <<"line_length= " << line_length << std::endl;
        
            //充电桩识别//
            if(MODEL_MIN_LENGTH <= line_length && line_length <= MODEL_MAX_LENGTH)
            {
                //计算充电桩位置//
                stand_pos_.x = (FittedLine.at(i).startPoint.x+FittedLine.at(i).endPoint.x)/2;
                stand_pos_.y = (FittedLine.at(i).startPoint.y+FittedLine.at(i).endPoint.y)/2;
                // std::cout << "stand_pos_.x" << stand_pos_.x << std::endl;
                // std::cout << "stand_pos_.y" << stand_pos_.y << std::endl;
                //充电桩到雷达的距离//
                chargestand2laser_dist = sqrt(pow(stand_pos_.x,2)+pow(stand_pos_.y,2));
                
                //判断执行回充还是执行开始工作//
                if(go_home_)
                {
                    if(chargestand2laser_dist > MAX_VALID_DIS)
                    {
                        continue;
                    }
            
                    else if(chargestand2laser_dist <= MAX_VALID_DIS && chargestand2laser_dist > MIN_VALID_DIS)
                    {
                        std::cout << "Finding the chargestand,start to recharge!" << std::endl;
                        //计算沿线走的直线的两个端点//
                        along_line_.startpoint.x = 0;
                        along_line_.startpoint.y = static_cast<double>(stand_pos_.y)/1000;
                        along_line_.endpoint.x = static_cast<double>(stand_pos_.x - MIN_VALID_DIS)/1000;
                        along_line_.endpoint.y = static_cast<double>(stand_pos_.y)/1000;

                        std::cout << "endpoint.x::" << along_line_.endpoint.x << std::endl;
                        std::cout << "endpoint.y::" << along_line_.endpoint.y << std::endl;
                        std::cout << "startpoint.y::" << along_line_.startpoint.y << std::endl;

                        finding_stand_ = true;
                    }
                    //完成沿直线行走//
                    if(finished_along_edge_)
                    {
                        //计算扫雪机到充电桩距离//
                        dist_to_stand_ = chargestand2laser_dist - 450;//400是雷达到扫雪车车尾的距离
                        std::cout << "dist_to_stand = " << dist_to_stand_ << std::endl;
                        //计算正交角//
                        orthogonal_angle_ = acos(((FittedLine.at(i).endPoint.x-FittedLine.at(i).startPoint.x)*(-stand_pos_.x)+
                        (FittedLine.at(i).endPoint.y-FittedLine.at(i).startPoint.y)*(-stand_pos_.y))/(line_length*chargestand2laser_dist));
                        yatNormalAngle(orthogonal_angle_);
                        std::cout << "orthogonal_angle_= " << orthogonal_angle_ << std::endl;

                        finished_along_edge_ = false;
                    }
                    // else 
                    // {
                    //     //计算扫雪机到充电桩距离//
                    //     dist_to_stand_ = chargestand2laser_dist - 360;//360是雷达到扫雪车车尾的距离
                    //     //计算正交角//
                    //     orthogonal_angle_ = acos(((FittedLine.at(i).endPoint.x-FittedLine.at(i).startPoint.x)*(-stand_pos_.x)+
                    //     (FittedLine.at(i).endPoint.y-FittedLine.at(i).startPoint.y)*(-stand_pos_.y))/(line_length*chargestand2laser_dist));
                    //     yatNormalAngle(orthogonal_angle_);
                    //     std::cout << "orthogonal_angle_= " << orthogonal_angle_ << std::endl;

                    //     // finding_stand_ = true;
                    // }
                }

                if(start_work_)
                {
                    if(chargestand2laser_dist < MAX_VALID_DIS)
                    {
                        std::cout << "Finding the chargestand,start to recharge!" << std::endl;
                        //计算沿线走的直线的两个端点//
                        along_line_.startpoint.x = 0.2;
                        along_line_.startpoint.y = static_cast<double>(stand_pos_.y)/1000;
                        along_line_.endpoint.x = 0.5;
                        along_line_.endpoint.y = static_cast<double>(stand_pos_.y)/1000;

                        std::cout << "endpoint.y::" << along_line_.endpoint.y << std::endl;
                        std::cout << "startpoint.y::" << along_line_.startpoint.y << std::endl;

                        finding_stand_ = true;
                    }
                } 
            }
            // else
            // {
            //     continue;
            // }
        }
        else
        {
            continue;
        }   
    }
}
/***********************************************************
    函数名称：yatNormalAngle
    函数功能：将角度归一化在-PI到PI之间
    入口参数：需要归一化的角度(指弧度)
    出口参数：无
    返回值: 返回值为归一化后的角度
    备 注：
***********************************************************/
double YATLaserRecharge::yatNormalAngle(double angle)
{
    double trueAngle = angle;
    //将大于PI的角度归一化在-PI到0之间//
    while (trueAngle > PI)
    {
        trueAngle -= 2*PI;
    }
    //将小于-PI的角度归一化在0到PI之间
    while (trueAngle < -PI)
    {
        trueAngle += 2*PI;
    }

    return trueAngle;   
}

/***********************************************************
    函数名称：yatDeltaAngle
    函数功能：计算目标角度与当前角度的归一化后的角度差值
    入口参数：targetAngle:目标角度 currentAngle:当前角度
    出口参数：无
    返回值: 返回值为归一化后的角度差值
    备 注：
***********************************************************/
double YATLaserRecharge::yatDeltaAngle(double targetAngle, double currentAngle)
{
    return yatNormalAngle(targetAngle-currentAngle);
}

/***********************************************************
    函数名称：yatDeltaDistance
    函数功能：计算直线起始点到终止点的距离
    入口参数：endPoint:终止点 startPoint:起始点
    出口参数：无
    返回值: 返回值为直线起始点到终止点的距离值
    备 注：
***********************************************************/
double YATLaserRecharge::yatDeltaDistance(YATPointXY endPoint,YATPointXY startPoint)
{
    //计算终止点到起始点坐标x和y的差值
    double delta_X = endPoint.x - startPoint.x;
    double delta_Y = endPoint.y - startPoint.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

/***********************************************************
    函数名称：yatDistanceToLine
    函数功能：计算机器人当前位姿到直线的距离
    入口参数：endPoint:直线终止点 startPoint:直线起始点 robotPose:机器人当前位姿
    出口参数：无
    返回值: 返回值为机器人当前位姿到直线的垂直距离
    备 注：
***********************************************************/
double YATLaserRecharge::yatDistanceToLine(YATPointXY endPoint,YATPointXY startPoint)
{
    // 计算直线起始点到终止点的距离
    double pathDistance=yatDeltaDistance(endPoint,startPoint);

    YATPointXY pathVector,normalVector,targetVector;

    //计算路径向量的单位向量//
    pathVector.x = (endPoint.x-startPoint.x)/pathDistance;
    pathVector.y = (endPoint.y-startPoint.y)/pathDistance;

    //计算路径向量的单位法向量// 
    normalVector.x = -pathVector.y;
    normalVector.y = pathVector.x;
    
    //计算目标向量,即路径直线终止点与机器人当前位姿的向量
    targetVector.x = endPoint.x - robot_pose_.x;
    targetVector.y = endPoint.y - robot_pose_.y;

    return (targetVector.x*normalVector.x+targetVector.y*normalVector.y);
}
/***********************************************************
    函数名称：yatSign
    函数功能：获取数据的正负
    入口参数：数据值
    出口参数：无
    返回值: 数据为正数时,返回1;负数时返回-1;零时返回0;
    备 注：
***********************************************************/
int YATLaserRecharge::yatSign(double data)
{
    if(data>0)
    {
        return 1;
    }
    else if (data<0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
int YATLaserRecharge::yatCalMoveDistance(double cur_mileage)
{
    static int move_distance = 0;
    ros::Time cur_time = ros::Time::now();
    static double last_mileage = cur_mileage;
    int delta_mileage =  static_cast<int>((cur_mileage - last_mileage)*1000);
    ROS_INFO("last_mileage,cur_mileage,and delta_mileage are %f, %f, %f",last_mileage,cur_mileage,delta_mileage);
    move_distance += delta_mileage;
    std::cout<<"move_distance =" << move_distance << std::endl;
    
    last_mileage = cur_mileage;
    return move_distance;
}
/***********************************************************
    函数名称：yatBackMotion
    函数功能：控制小车沿"边"行走,即控制小车沿直线后退行走
    入口参数：nextPoint:路径直线的起点,lastPoint:路径直线的终点
    出口参数：无
    返回值: 若小车当前位姿距路径直线的终点小于一定阈值,返回true;否则返回false
    备 注：
***********************************************************/
bool YATLaserRecharge::yatBackMotion(YATPointXY nextPoint,YATPointXY lastPoint)
{   
    //计算直线起始点到终止点的距离//
    double pathDistance = yatDeltaDistance(nextPoint,lastPoint); 
    
    //直线的单位向量//
    Eigen::Vector2d pathVector((nextPoint.x-lastPoint.x)/pathDistance,(nextPoint.y-lastPoint.y)/pathDistance);
    //直线的单位法向量//
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    //目标点与机器人当前位姿的目标向量//
    Eigen::Vector2d targetVector(nextPoint.x - robot_pose_.x,nextPoint.y - robot_pose_.y);

    //计算机器人当前位姿到直线的距离 距离为正 表明机器人在直线右侧, 负为左侧//
    double trajectoryDistance = yatDistanceToLine(nextPoint,lastPoint);  // left is negative; right is position;
    double PathAngle = atan2( nextPoint.y - lastPoint.y,nextPoint.x - lastPoint.x);
    //预瞄距离//
    double aimLength = 0.5;//when aimLength =0.3,The actual test result is not ideal,0.5 is better  
    // ROS_INFO("PathAngle = %f",PathAngle);

    //控制角度//
    double controlAngle = 0.5*PI*yatSign(trajectoryDistance);
    // double controlAngle = -0.5*PI*yatSign(trajectoryDistance);
    
    if( fabs( trajectoryDistance ) < aimLength)
    {
        //计算控制角度//
        controlAngle = asin(trajectoryDistance / aimLength);
        // controlAngle = -asin(trajectoryDistance / aimLength);
    }
    //机器人的目标朝向//
    double targetRobotPose = yatNormalAngle(controlAngle + PathAngle);
    //当前朝向与目标朝向的差值//
    double controlObject = yatDeltaAngle(targetRobotPose,robot_pose_.z);

    YATPointXY commandVelocity;
    
    //角速度//
    commandVelocity.y = -controlObject *1.2 /PI;//后退的转向 修改:2020-06-19
    // commandVelocity.y = controlObject *1.2 /PI;//前进转向 

    // ROS_INFO("commandVelocity.y = %f,controlObject= %f,robotpose_.z = %f",commandVelocity.y,controlObject,robotpose_.z);

    if (fabs(commandVelocity.y) > 0.5)
    {
        commandVelocity.y = 0.5 * yatSign(commandVelocity.y);
    }
    
    // 机器人当前位姿到目标点的距离//
    double deadLineDistance = targetVector.transpose() * pathVector;
    ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose);
    
    //线速度//
    // commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);//正向或前进
    commandVelocity.x = -0.5 * deadLineDistance * cos(controlObject);//反向或后退

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * yatSign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * yatSign(commandVelocity.x);
    }
    

    if(deadLineDistance > 0.05)
    {
        cmd_vel_msg_.linear.x = commandVelocity.x;
        cmd_vel_msg_.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.angular.z = 0;
        finished_along_edge_ = true;
        return true;

    }

}
/***********************************************************
    函数名称：yatForwardMotion
    函数功能：控制小车沿"边"行走,即控制小车沿直线前进行走
    入口参数：nextPoint:路径直线的起点,lastPoint:路径直线的终点
    出口参数：无
    返回值: 若小车当前位姿距路径直线的终点小于一定阈值,返回true;否则返回false
    备 注：修改于2020-06-19
***********************************************************/
bool YATLaserRecharge::yatForwardMotion(YATPointXY nextPoint,YATPointXY lastPoint)
{   
    //计算直线起始点到终止点的距离//
    double pathDistance = yatDeltaDistance(nextPoint,lastPoint); 
    
    //直线的单位向量//
    Eigen::Vector2d pathVector((nextPoint.x-lastPoint.x)/pathDistance,(nextPoint.y-lastPoint.y)/pathDistance);
    //直线的单位法向量//
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    //目标点与机器人当前位姿的目标向量//
    Eigen::Vector2d targetVector(nextPoint.x - robot_pose_.x,nextPoint.y - robot_pose_.y);

    //计算机器人当前位姿到直线的距离 距离为正 表明机器人在直线右侧, 负为左侧//
    double trajectoryDistance = yatDistanceToLine(nextPoint,lastPoint);  // left is negative; right is position;
    double PathAngle = atan2( nextPoint.y - lastPoint.y,nextPoint.x - lastPoint.x);
    //预瞄距离//
    double aimLength = 0.5;//when aimLength =0.3,The actual test result is not ideal,0.5 is better  
    // ROS_INFO("PathAngle = %f",PathAngle);

    //控制角度//
    double controlAngle = 0.5*PI*yatSign(trajectoryDistance);
    // double controlAngle = -0.5*PI*yatSign(trajectoryDistance);
    
    if( fabs( trajectoryDistance ) < aimLength)
    {
        //计算控制角度//
        controlAngle = asin(trajectoryDistance / aimLength);
        // controlAngle = -asin(trajectoryDistance / aimLength);
    }
    //机器人的目标朝向//
    double targetRobotPose = yatNormalAngle(controlAngle + PathAngle);
    //当前朝向与目标朝向的差值//
    double controlObject = yatDeltaAngle(targetRobotPose,robot_pose_.z);

    YATPointXY commandVelocity;
    
    //角速度//
    commandVelocity.y = -controlObject *1.2 /PI;
    // commandVelocity.y = controlObject *1.2 /PI;

    // ROS_INFO("commandVelocity.y = %f,controlObject= %f,robotpose_.z = %f",commandVelocity.y,controlObject,robotpose_.z);

    if (fabs(commandVelocity.y) > 0.5)
    {
        commandVelocity.y = 0.5 * yatSign(commandVelocity.y);
    }
    
    // 机器人当前位姿到目标点的距离//
    double deadLineDistance = targetVector.transpose() * pathVector;
    ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose);
    
    //线速度//
    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);//正向或前进
    // commandVelocity.x = -0.5 * deadLineDistance * cos(controlObject);//反向或后退

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * yatSign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * yatSign(commandVelocity.x);
    }
    

    if(deadLineDistance > 0.05)
    {
        cmd_vel_msg_.linear.x = commandVelocity.x;
        cmd_vel_msg_.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.angular.z = 0;
        return true;
    }
}

/***********************************************************
    函数名称：yatMotion
    函数功能：实现小车的控制状态之间的转换
    入口参数：无
    出口参数：无
    返回值:若小车沿直线行走时到达直线的终止点,返回true,否则false
    备 注：
***********************************************************/
bool YATLaserRecharge::yatGoHomeMotion()
{
    std::cout << "/***************go home****************/" << std::endl;
    static int motionStep = 0;
    bool taskFinish = false;
    static int distance_threshold = 300;//5cm
    int move_dis = 0;

    // static cv::Point3f curPoint = robot_pose_;
    static YATPointXYZ curPoint = robot_pose_;
    static YATPointXY startPoint = along_line_.startpoint; 
    static YATPointXY endPoint = along_line_.endpoint;
    YATLine tempara;

    //判断机器人是否停止//
    // bool robotStop = ((fabs(odomdata_.linearVel) < 0.02)&&(fabs(odomdata_.angularVel) < 0.02))?true:false;

    if(motionStep == 0)
    {
        //判断是否识别到充电桩//
        if(finding_stand_)
        {
            motionStep = 1;
        }
        else
        {
            cmd_vel_msg_.linear.x = 0;
            cmd_vel_msg_.angular.z = 0;
            return false;
        }
    }

    if(motionStep == 1)
    {
        //利用临时变量存储直线的端点//
        tempara = along_line_;
        startPoint.x = tempara.startpoint.x;
        startPoint.y = tempara.startpoint.y;
        endPoint.x = tempara.endpoint.x;
        endPoint.y = tempara.endpoint.y;

        ROS_INFO("startPoint is %f, %f,endPoint is %f, %f",startPoint.x,startPoint.y, endPoint.x, endPoint.y);
        ROS_INFO("robotpose is %f, %f, %f",robot_pose_.x,robot_pose_.y,robot_pose_.z);
        curPoint = robot_pose_;

        motionStep = 2;  
    }

    if(motionStep == 2)
    {
        //开始沿直线行走//
        if(yatBackMotion(endPoint,startPoint))
        {
            motionStep = 3;
        }
        
        //充电座位置更新//
        if(tempara.startpoint.y != along_line_.startpoint.y || tempara.endpoint.x != along_line_.endpoint.x)
        {
            motionStep = 1;
        }
    }

    if(motionStep == 3)
    {
        //到达直线的终止点,机器人停止,判断两向量是否正交//
        if(MIN_ANGLE <= orthogonal_angle_ && orthogonal_angle_ <= MAX_ANGLE)
        {
            motionStep = 4;
        }

        // else
        // {
        //     startPoint.x = 0;
        //     startPoint.y = static_cast<double>(stand_pos_.y)/1000;
        //     endPoint.x = static_cast<double>(stand_pos_.x - MIN_VALID_DIS)/1000;
        //     endPoint.y = static_cast<double>(stand_pos_.y)/1000;

        //     ROS_INFO("startPoint is %f, %f,endPoint is %f, %f",startPoint.x,startPoint.y, endPoint.x, endPoint.y);
        //     curPoint = robot_pose_;

        //     motionStep = 2;
        // }

        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.angular.z = 0;
    }

    if(motionStep == 4)
    {
        // ros::Time current_time = ros::Time::now();
        // static double last_mileage = center_mileage_;
        // int delta_dis =  static_cast<int>((center_mileage_ - last_mileage)*1000);
        // move_dis += delta_dis;
        // std::cout << "last_mileage" << last_mileage <<std::endl;
        // std::cout << "center_mileage_" << center_mileage_ <<std::endl;
        // std::cout << " move_dis:: " << move_dis << std::endl;
        // std::cout << "dist_to_stand = " << dist_to_stand_ << std::endl;

        // //保存上一时刻数据//
        // last_mileage = center_mileage_;
        move_dis = yatCalMoveDistance(center_mileage_);

        if((abs(move_dis) +distance_threshold) < dist_to_stand_)
        {
            cmd_vel_msg_.linear.x = -0.2;
            cmd_vel_msg_.angular.z = 0;
        }

        else
        {
            cmd_vel_msg_.linear.x = 0;
            cmd_vel_msg_.angular.z = 0;
            taskFinish = true;
            
        }

        //保存上一时刻数据//
        // last_mileage = center_mileage_;
    }

    ROS_INFO("along boundary -> motion step is : %d.",motionStep);

    return taskFinish;
}
/***********************************************************
    函数名称：yatStartWorkMotion
    函数功能：实现小车出库开始工作的控制状态之间的转换
    入口参数：无
    出口参数：无
    返回值:若小车沿直线行走时到达直线的终止点,返回true,否则false
    备 注：
***********************************************************/
bool YATLaserRecharge::yatStartWorkMotion()
{
    std::cout << "/***************start work****************/" << std::endl;
    static int motionStep = 0;
    bool taskFinish = false;
    int move_dis = 0;
    
    static YATPointXYZ curPoint = robot_pose_;
    static YATPointXY startPoint = along_line_.startpoint; 
    static YATPointXY endPoint = along_line_.endpoint;
    YATLine tempara;

    if(motionStep == 0)
    {
        //判断是否识别到充电桩//
        if(finding_stand_)
        {
            motionStep = 1;
        }
        else
        {
            cmd_vel_msg_.linear.x = 0;
            cmd_vel_msg_.angular.z = 0;
            return false;
        }
    }

    if(motionStep == 1)
    {
        //利用临时变量存储直线的端点//
        tempara = along_line_;
        startPoint.x = tempara.startpoint.x;
        startPoint.y = tempara.startpoint.y;
        endPoint.x = tempara.endpoint.x;
        endPoint.y = tempara.endpoint.y;

        ROS_INFO("startPoint is %f, %f,endPoint is %f, %f",startPoint.x,startPoint.y, endPoint.x, endPoint.y);
        ROS_INFO("robotpose is %f, %f, %f",robot_pose_.x,robot_pose_.y,robot_pose_.z);
        curPoint = robot_pose_;

        motionStep = 2;  
    }

    if(motionStep == 2)
    {
        //开始沿直线行走//
        if(yatForwardMotion(endPoint,startPoint))
        {
            motionStep = 3;
        }
        
        //充电座位置更新//
        if(tempara.startpoint.y != along_line_.startpoint.y || tempara.endpoint.x != along_line_.endpoint.x)
        {
            motionStep = 1;
        }
    }

    if(motionStep == 3)
    {
        move_dis = yatCalMoveDistance(center_mileage_);
        bool robot_stop = (move_dis = 0)?true:false;
        //到达直线的终止点,机器人停止,判断两向量是否正交//
        if(robot_stop)
        {
            taskFinish = true;
        }

        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.angular.z = 0;
    }

    ROS_INFO("along boundary -> motion step is : %d.",motionStep);

    return taskFinish;
}
/***********************************************************
    函数名称：yatExecute
    函数功能：运行回充模块程序,识别充电座进行自动回充
    入口参数: 
    出口参数：
    备 注：
***********************************************************/
void YATLaserRecharge::yatExecute()
{
   // 测试提取出的雷达数据拟合的直线//
    // IplImage* LaserImage = cvCreateImage(cvSize(RADAR_IMAGE_WDITH,RADAR_IMAGE_HEIGHT),IPL_DEPTH_8U,3);
    // cvNamedWindow("BreakedRadar",1);
    // cvNamedWindow("Radar",1);
    // int lineCnt = 0;
    // char key;

    // while (ros::ok())
    // {
    //     //显示原始雷达数据经过预处理后的有效数据//
    //     yatCreateRadarImage(LaserImage,use_scan_data_,use_angle_data_);
    //     cvShowImage("Radar",LaserImage);
    //     //对预处理后得到有效数据进行分割操作//
    //     yatRegionSegmentation();
    //     //对多边形拟合得到的直线进行分割//
    //     lineCnt = yatBreakPolyLine();
    //     //显示分割后的雷达数据//
    //     yatCreateRadarImage(LaserImage,break_scan_data_,break_angle_data_);
    //     //对分割后的雷达数据进行直线拟合//
    //     yatFitLine(FittedLine,break_scan_data_,break_angle_data_);
    //     yatChargingStandIdentification(FittedLine);
    //     //绘制拟合的直线//
    //     yatDrawRadarLine(FittedLine,LaserImage);
    //     cvShowImage("BreakedRadar",LaserImage);
    //     if(yatMotion())
    //     {
    //         std::cout << "Complete the recharge task!" << std::endl;
    //     }

    //     cmd_pub_.publish(cmd_vel_msg_);
    
    //     ros::spinOnce();

    //     key = cvWaitKey(5);
    // }

    // cvReleaseImage(&LaserImage);
    // cvDestroyWindow("BreakedRadar");
    // cvDestroyWindow("Radar");

    if( go_home_ == true || start_work_ == true)
    {
        //对预处理后得到有效数据进行分割操作//
        yatRegionSegmentation();
        //对多边形拟合得到的直线进行分割//
        yatBreakPolyLine();
        //对分割后的雷达数据进行直线拟合//
        yatFitLine(FittedLine,break_scan_data_,break_angle_data_);
        //充电座识别//
        yatChargingStandIdentification(FittedLine);
        //根据bool的值来判断执行回充还是移动到距离充电桩一定距离的位置//
        if(go_home_)
        {
            //回充运动控制//
            if(yatGoHomeMotion())
            {
                go_home_ = false;
                std::cout << "Complete the recharge task!" << std::endl;
            }
        }

        if(start_work_)
        {
            if(yatStartWorkMotion())
            {
                start_work_ =false;
                std::cout << "Finished the start_work task!" << std::endl;
            }
        }
        //发布控制扫雪机移动的速度//
        cmd_pub_.publish(cmd_vel_msg_);
    } 
}
