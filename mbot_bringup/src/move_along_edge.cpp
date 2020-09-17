#include "mbot_bringup/move_along_edge.h"

YATMoveAlongEdge::YATMoveAlongEdge()
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    robot_pose_sub_=nh_.subscribe("odom",1,&YATMoveAlongEdge::yatPoseCallback,this);
    plan_point_sub_=nh_.subscribe("planpoints",1,&YATMoveAlongEdge::yatPlanPointsCallback,this);

    yatIsUpdatePLan_ = false;
}
YATMoveAlongEdge::~YATMoveAlongEdge()
{

}

/***********************************************************
    函数名称：yatNormalAngle
    函数功能：将角度归一化在-PI到PI之间
    入口参数：需要归一化的角度(指弧度)
    出口参数：无
    返回值: 返回值为归一化后的角度
    备 注：
***********************************************************/
double YATMoveAlongEdge::yatNormalAngle(double angle)
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
double YATMoveAlongEdge::yatDeltaAngle(double targetAngle, double currentAngle)
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
double YATMoveAlongEdge::yatDeltaDistance(YATPointXY endPoint,YATPointXY startPoint)
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
double YATMoveAlongEdge::yatDistanceToLine(YATPointXY endPoint,YATPointXY startPoint,YATPointXYZ robotPose)
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
    targetVector.x = endPoint.x - robotPose.x;
    targetVector.y = endPoint.y - robotPose.y;

    return (targetVector.x*normalVector.x+targetVector.y*normalVector.y);
}

/***********************************************************
    函数名称：calParallelLine
    函数功能：根据距离阈值计算接收到的直线的平行线
    入口参数：endPoint:接收的直线终止点 startPoint:接收的直线起始点 d:到直线的距离阈值
    出口参数：无
    返回值: 返回值为包含平行线的终止点和起始点的结构体
    备 注：
***********************************************************/
YATLine YATMoveAlongEdge::yatCalParallelLine(YATPointXY endPoint,YATPointXY startPoint,double d)
{
    YATLine tempara;
    YATPointXY deltaValue;
    // double angle =atan2(-(endPoint.x-startPoint.x),endPoint.y-startPoint.y);
    // double angle =atan2(endPoint.x-startPoint.x,endPoint.y-startPoint.y);
    
    //计算直线端点的角度//
    double angle =atan2(endPoint.y-startPoint.y,endPoint.x-startPoint.x);
    //角度归一化//
    yatNormalAngle(angle);

    ROS_INFO("angle = %f",angle);
    
    //计算距直线距离为d的平行线的端点坐标与直线的端点坐标的差值//
    deltaValue.x = d *sin(angle);
    deltaValue.y = -d *cos(angle);

    //计算平行线的端点//
    tempara.startPoint.x=startPoint.x+deltaValue.x;
    tempara.startPoint.y=startPoint.y+deltaValue.y;
    tempara.endPoint.x =endPoint.x+deltaValue.x;
    tempara.endPoint.y =endPoint.y+deltaValue.y;

    ROS_INFO("delta_x=%f,delta_y=%f",deltaValue.x,deltaValue.y);

    ROS_INFO("tempara.startPoint.x = %f,tempara.startPoint.y = %f, tempara.endPoint.x = %f, tempara.endPoint.y = %f",tempara.startPoint.x,tempara.startPoint.y,tempara.endPoint.x,tempara.endPoint.y);

    return tempara;  
}

/***********************************************************
    函数名称：yatSign
    函数功能：获取数据的正负
    入口参数：数据值
    出口参数：无
    返回值: 数据为正数时,返回1;负数时返回-1;零时返回0;
    备 注：
***********************************************************/
int YATMoveAlongEdge::yatSign(double data)
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

/***********************************************************
    函数名称：yatPoseCallback
    函数功能：获取里程计相关数据,包括里程计位姿以及线速度和角速度
    入口参数：里程计消息指针
    出口参数：无
    备 注：
***********************************************************/
void YATMoveAlongEdge::yatPoseCallback(const nav_msgs::OdometryConstPtr& pose_msg)
{
    //获取机器人的位置//
    robotpose_.x=pose_msg->pose.pose.position.x;
    robotpose_.y=pose_msg->pose.pose.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose_msg->pose.pose.orientation,quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    
    //获取机器人的角度(或朝向)//
    robotpose_.z=yaw;
    
    //获取机器人的角速度和线速度(里程计发布出的)//
    odomdata_.linearVel=pose_msg->twist.twist.linear.x;
    odomdata_.angularVel=pose_msg->twist.twist.angular.z;
}

/***********************************************************
    函数名称：yatPlanPointsCallback
    函数功能：获取路径点的坐标信息x和y
    入口参数：路径点消息指针
    出口参数：无
    备 注：
***********************************************************/
void YATMoveAlongEdge::yatPlanPointsCallback(const geometry_msgs::PoseArrayConstPtr& planpoints_msg)
{
    //获取直线的坐标点的原始信息并将单位转化为m//
    origline_.startPoint.x = planpoints_msg->poses[0].position.x/100.0;
    origline_.startPoint.y = planpoints_msg->poses[0].position.y/100.0;
    origline_.endPoint.x = planpoints_msg->poses[1].position.x/100.0;
    origline_.endPoint.y = planpoints_msg->poses[1].position.y/100.0;

    // ROS_INFO("origpaorigline_ra.startPoint is %f, %f,origline_.endPoint is %f,%f",origline_.startPoint.x,origline_.startPoint.y,
    // origline_.endPoint.x,origline_.endPoint.y);
    
    //计算直线的平行线//
    parallelline_ = yatCalParallelLine(origline_.endPoint,origline_.startPoint,0.5);
    // parallelline_ = yatCalParallelLine(origline_.endPoint,origline_.startPoint,0.7);
   
    //计算转换到里程计坐标系后的直线信息//
    transline_.startPoint.x = parallelline_.startPoint.x+0.115+robotpose_.x;
    transline_.startPoint.y = -parallelline_.startPoint.y-0.145+robotpose_.y;
    transline_.endPoint.x = parallelline_.endPoint.x+0.115+robotpose_.x;
    transline_.endPoint.y = -parallelline_.endPoint.y-0.145+robotpose_.y;
    
    //若原始直线的坐标y始终为0,表明路径未更新//
    if(origline_.startPoint.y==0&&origline_.endPoint.y==0)
    {
        yatIsUpdatePLan_ = false;
    }

    else
    {
        yatIsUpdatePLan_ =true;
    }
}

/***********************************************************
    函数名称：yatAlongEdgeMotion
    函数功能：控制小车沿"边"行走,即控制小车沿直线行走
    入口参数：nextPoint:路径直线的起点,lastPoint:路径直线的终点
    出口参数：无
    返回值: 若小车当前位姿距路径直线的终点小于一定阈值,返回true;否则返回false
    备 注：
***********************************************************/
bool YATMoveAlongEdge::yatAlongEdgeMotion(YATPointXY nextPoint,YATPointXY lastPoint)
{   
    //计算直线起始点到终止点的距离//
    double pathDistance = yatDeltaDistance(nextPoint,lastPoint); 
    
    //直线的单位向量//
    Eigen::Vector2d pathVector((nextPoint.x-lastPoint.x)/pathDistance,(nextPoint.y-lastPoint.y)/pathDistance);
    //直线的单位法向量//
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    //目标点与机器人当前位姿的目标向量//
    Eigen::Vector2d targetVector(nextPoint.x - robotpose_.x,nextPoint.y - robotpose_.y);

    
    //计算机器人当前位姿到直线的距离 距离为正 表明机器人在直线右侧, 负为左侧//
    double trajectoryDistance = yatDistanceToLine(nextPoint,lastPoint,robotpose_);  // left is negative; right is position;
    double PathAngle = atan2( nextPoint.y - lastPoint.y,nextPoint.x - lastPoint.x);
    //预瞄距离//
    double aimLength = 0.5;//when aimLength =0.3,The actual test result is not ideal,0.5 is better  

    // ROS_INFO("PathAngle = %f",PathAngle);

    //控制角度//
    double controlAngle = 0.5*PI*yatSign(trajectoryDistance);
    
    if( fabs( trajectoryDistance ) < aimLength)
    {
        //计算控制角度//
        controlAngle = asin(trajectoryDistance / aimLength);
    }
    //机器人的目标朝向//
    double targetRobotPose = yatNormalAngle(controlAngle + PathAngle);
    //当前朝向与目标朝向的差值//
    double controlObject = yatDeltaAngle(targetRobotPose,robotpose_.z);

    YATPointXY commandVelocity;
    
    //角速度//
    commandVelocity.y = controlObject *1.2 /PI;
    // commandVelocity.y = controlObject *1.2 /PI;

    // ROS_INFO("commandVelocity.y = %f,controlObject= %f,robotpose_.z = %f",commandVelocity.y,controlObject,robotpose_.z);

    if (fabs(commandVelocity.y) > 0.5)
    {
        commandVelocity.y = 0.5 * yatSign(commandVelocity.y);
    }
    
    // 机器人当前位姿到目标点的距离//
    double deadLineDistance = targetVector.transpose() * pathVector;
    // ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose);
    
    //线速度//
    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

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
        cmd_vel_msg_.angular.z = 0;;
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
bool YATMoveAlongEdge::yatMotion()
{
    static int motionStep = 0;
    bool taskFinish = false;

    static YATPointXYZ curPoint = robotpose_;
    static YATPointXY startPoint =transline_.startPoint;
    static YATPointXY endPoint = transline_.endPoint;
    static double targetAngle = 0;
    static double lineAngle =0;
    YATLine tempara;
    bool robotStop = ((fabs(odomdata_.linearVel) < 0.02)&&(fabs(odomdata_.angularVel) < 0.02))?true:false;

    if(motionStep==0)
    {
        //路径点更新//
        if(yatIsUpdatePLan_)
        {
            motionStep=1;
        }
        else
        {
            cmd_vel_msg_.linear.x=0;
            cmd_vel_msg_.angular.z=0;
            return false;
        }
    }

    if(motionStep == 1)
    {
        tempara = transline_;
        startPoint.x = tempara.startPoint.x;
        startPoint.y = tempara.startPoint.y;
        endPoint.x = tempara.endPoint.x;
        endPoint.y = tempara.endPoint.y;

        // ROS_INFO("startPoint is %f, %f,endPoint is %f, %f",startPoint.x,startPoint.y, endPoint.x, endPoint.y);
        // ROS_INFO("robotpose is %f, %f, %f",robotpose_.x,robotpose_.y,robotpose_.z);
        curPoint = robotpose_;

        motionStep=2;  
    }

    if(motionStep==2)
    {
        //开始沿直线行走
        if(yatAlongEdgeMotion(endPoint,startPoint))
        {
            motionStep =3;
        }
        
        //路径点更新
        if(tempara.startPoint.x!=transline_.startPoint.x||tempara.startPoint.x!=transline_.startPoint.x)
        {
            motionStep =0;
        }
    }

    if(motionStep==3)
    {
        //到达直线的终止点,机器人停止
        if(robotStop)
        {
            motionStep = 0;
            taskFinish =true;
        }

        cmd_vel_msg_.linear.x=0;
        cmd_vel_msg_.angular.z=0;
    }

    ROS_INFO("along boundary -> motion step is : %d.",motionStep);

    return taskFinish;
    
}

/***********************************************************
    函数名称：yatExecute
    函数功能：执行小车的整体控制程序,实现小车的运动控制
    入口参数：无
    出口参数：无
    备 注：
***********************************************************/
void YATMoveAlongEdge::yatExecute()
{
    if(yatMotion())
    {
        if(origline_.startPoint.y == 0||origline_.endPoint.y ==0)
        {
            cmd_vel_msg_.linear.x=0;
            cmd_vel_msg_.angular.z=0;

            ROS_INFO("waiting for new plan points!");
        }
    }

    cmd_vel_pub_.publish(cmd_vel_msg_);
}







