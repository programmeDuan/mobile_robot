//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

//map
//#include <contour.h>
#include <path.h>

// #include <mapping.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

//Synchronize subscription messages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include  "navigateFunction.cpp"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                        geometry_msgs::PoseStamped> sync_policy_classifiction;

//get the current date //date -s "20180929 08:00:00"
std::string getDate();

//point file
std::string point_path = "/home/rasp008/mower_ws/src/gps_zigzag/data/";
// std::string point_path = "/home/yat/catkin_ws/src/gps_zigzag/data/";
std::string file_status = point_path + getDate() + "sensor.txt";
std::ofstream fout_point(file_status.c_str());

/******************* ros publisher ****************************/
ros::Publisher pose_pub;

bool motionFinish = false;

DETECT_TYPE detectData;
CONFIG_PARAMETER configData;
SOLUTION_PARAMETER solutionData;

void initialParameter(CONFIG_PARAMETER &configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    POINT_XYZ zeroPoint;
    zeroPoint.x = 0;
    zeroPoint.y = 0;
    zeroPoint.z = 0;
    
    configMessage.baseENU = zeroPoint; // origin point , coordinates is (0,0,0)

    configMessage.enableBase= false;
    solutionMessage.initialState = false;
}

void attitudeFilterSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // ROS_INFO("attitude kalman filter delta time is: %5.15f",dt);

    // update state transition matrix
    Eigen::Matrix<double,2,2> PHI;
    PHI.setZero(2,2);
    PHI(0,0) = 1;
    PHI(0,1) = - dt;
    PHI(1,1) = 1;

    // double currentYaw = configMessage.euler.z;
    // static double yaw_prev = currentYaw; 
    // double deltaYaw = deltaAngle(currentYaw,yaw_prev);  // delta yaw measured by imu
    // yaw_prev = currentYaw; 
    //x(k)=A*x(k-1)+B*u(k)

    // update priori estimate yaw states 
    solutionMessage.robotEuler.x = configMessage.euler.x;
    solutionMessage.robotEuler.y = configMessage.euler.y;
    solutionMessage.robotEuler.z = solutionMessage.robotEuler.z + dt * (configMessage.gyro.z - configMessage.gyroBias.z); //deltaYaw; // dt * configMessage.gyro.z; //

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,2,2> P0;
    P0.setZero(2,2);
    P0(0,0) = pow(DegreeToRad*0.2,2); // initial gyro angle error
    P0(1,1) = pow(DegreeToRad/64/50,2); // initial gyro angle rate error

    static Eigen::Matrix<double,2,2> P_p = P0;

    // configure system state Noise covariance Q
    Eigen::Matrix<double,2,2> Q;
    Q.setZero(2,2);
    Q(0,0) = pow(DegreeToRad * 0.02,2); // gyro angle random noise
    Q(1,1) = pow(DegreeToRad * 0.08/3600,2);  // gyro angle rate random noise

    // configure Observation matrix
    Eigen::Matrix<double,1,2> H;
    H.setZero(1,2);
    H(0,0) = 1;

    // configure Observation noise covariance R
    Eigen::Matrix<double,1,1> R;
    R.setZero(1,1);
    R(0,0) = pow(15*DegreeToRad,2); // GPS angle random noise

    Eigen::Matrix<double,2,1> K;
    K.setZero(2,1);

    /**************************************************************/
    bool disableGPS = (sqrtValue(configMessage.gnssVel)<0.2)?true:false;
    bool disableOdom = (fabs(configData.odomVel.z)<0.2)?true:false;

    // posteriori evaluation
    double yawGPS = constrainAngle(atan2(configMessage.gnssVel.y,configMessage.gnssVel.x) - PI / 2);
    double deltaTheta = deltaAngle(yawGPS,solutionMessage.robotEuler.z); 
    if(disableGPS || disableOdom || (!configData.updateGNSS) || (fabs(deltaTheta) > 60 * DegreeToRad) )
    {
        yawGPS = solutionMessage.robotEuler.z;
    }
    else
    {
        // update Prior estimate covariance (PHI=A)
        P_p = PHI * P_p * PHI.transpose() + Q;

        // update kalman gain
        K = P_p*H.transpose()*( H*P_p*H.transpose() + R).inverse();

        // configure unit matrix
        Eigen::Matrix<double,2,2> I;
        I.setIdentity(2,2);

        // update posteriori estimate covariance
        Eigen::Matrix<double,2,2> P_f = (I - K*H)*P_p;
        P_p = P_f;
    }
    /**************************************************************/

    Eigen::Matrix<double,1,1> Y;
    Y(0,0) = deltaTheta;

    // update posteriori evaluation states
    Eigen::Matrix<double,2,1> X_f = K * Y;

    solutionMessage.robotEuler.z = constrainAngle(solutionMessage.robotEuler.z + X_f(0,0));
    solutionMessage.gyroBias.z = solutionMessage.gyroBias.z + X_f(1,0);
}

void positionFilterSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    // configure unit matrix
    Eigen::Matrix<double,4,4> I;
    I.setIdentity(4,4);

    // update state transition matrix
    Eigen::Matrix<double,4,4> PHI;
    PHI.setZero(4,4);
    PHI(0,2) = dt;
    PHI(1,3) = dt;
    PHI += I;

    Eigen::Matrix<double,2,2> Cbn;
    Cbn(0,0) = cos(solutionMessage.robotEuler.z);
    Cbn(0,1) = - sin(solutionMessage.robotEuler.z);
    Cbn(1,0) = sin(solutionMessage.robotEuler.z);
    Cbn(1,1) = cos(solutionMessage.robotEuler.z);

    double currentMileage = 0.5*(configMessage.odomVel.x + configMessage.odomVel.y);  // current mileage measured by odom
    static double mileage_prev = currentMileage; 
    double deltaMileage = currentMileage - mileage_prev;
    mileage_prev = currentMileage;//???

    Eigen::Vector2d positionBody(0,deltaMileage);
    Eigen::Vector2d velocityBody(0,configData.odomVel.z);

    Eigen::Vector2d positionNav = Cbn * positionBody;
    Eigen::Vector2d velocityNav = Cbn * velocityBody;

    solutionData.robotEKF.x = solutionData.robotEKF.x + positionNav(0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + positionNav(1);
    solutionData.robotVel.x = velocityNav(0);
    solutionData.robotVel.y = velocityNav(1);

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,4,4> P0;
    P0.setZero(4,4);
    P0(0,0) = pow( 0.1 , 2 ); // initial X axis position noise
    P0(1,1) = pow( 0.1 , 2 ); // initial Y axis position noise
    P0(2,2) = pow( 0.25 , 2 ); // initial X axis velocity noise
    P0(3,3) = pow( 0.25 , 2 ); // initial Y axis velocity noise

    // configure system state Noise covariance Q
    Eigen::Matrix<double,4,4> Q;
    Q.setZero(4,4);
    Q(0,0) = pow( 0.0025 , 2 );  // X axis position noise of Odom
    Q(1,1) = pow( 0.0025 , 2 );  // Y axis position noise of Odom
    Q(2,2) = pow( 0.03125 , 2 );  // X axis velocity noise of Odom
    Q(3,3) = pow( 0.03125 , 2 );  // Y axis velocity noise of Odom

    // update Prior estimate covariance
    static Eigen::Matrix<double,4,4> P_p = P0;

    // configure Observation matrix
    Eigen::Matrix<double,4,4> H;
    H.setZero(4,4);
    H = I;

    // configure Observation noise covariance R
    Eigen::Matrix<double,4,4> R;
    R.setZero(4,4);
    R(0,0) = pow( 2.5 , 2 );  // X axis position noise of GNSS
    R(1,1) = pow( 2.5 , 2 );  // Y axis position noise of GNSS
    R(2,2) = pow( 0.2 , 2 );  // X axis velocity noise of GNSS
    R(3,3) = pow( 0.2 , 2 );  // Y axis velocity noise of GNSS

    Eigen::Matrix<double,4,4> K;
    K.setZero(4,4);

    if(configMessage.updateGNSS)
    {
        P_p = PHI * P_p * PHI.transpose() + Q;
        // for(int i = 0;i<4;i++)
        // {
        //     for(int j = 0;j<4;j++)
        //     {
        //         ROS_INFO("kalman filter P_p(%d,%d) = %f",i,j,P_p(i,j));
        //     }
        // }

        Eigen::Matrix<double,4,4> S = H*P_p*H.transpose() + R;
        // for(int i = 0;i<4;i++)
        // {
        //     for(int j = 0;j<4;j++)
        //     {
        //         ROS_INFO("kalman filter S(%d,%d) = %f",i,j,S(i,j));
        //     }
        // }

        // update kalman gain
        K = P_p*H.transpose()*(H*P_p*H.transpose() + R).inverse();

    }

    Eigen::Matrix<double,4,1> Y;
    Y.setZero(4,1);
    Y(0,0) = configMessage.gnssENU.x - solutionData.robotEKF.x;
    Y(1,0) = configMessage.gnssENU.y - solutionData.robotEKF.y;
    Y(2,0) = configMessage.gnssVel.x - solutionData.robotVel.x;
    Y(3,0) = configMessage.gnssVel.y - solutionData.robotVel.y;

    // update posteriori evaluation states
    Eigen::Matrix<double,4,1> X_f = K * Y;

    // update posteriori estimate covariance
    Eigen::Matrix<double,4,4> P_f = (I - K*H)*P_p;
    P_p = P_f;

    solutionData.robotEKF.x = solutionData.robotEKF.x + X_f(0,0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + X_f(1,0);
    solutionData.robotVel.x = solutionData.robotVel.x + X_f(2,0);
    solutionData.robotVel.y = solutionData.robotVel.y + X_f(3,0);
}

void deadReckoningSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    // double currentYaw = configMessage.euler.z;
    double currentYaw = solutionMessage.robotEuler.z; 
    double currentMileage = configMessage.odomMil.z;// current mileage measured by odom
	
    static double yaw_prev = currentYaw; 
    static double mileage_prev = currentMileage; 

    POINT_XYZ deltaTheta;
    deltaTheta.x = deltaAngle(configMessage.euler.x,solutionMessage.robotPose.x);
    deltaTheta.y = deltaAngle(configMessage.euler.y,solutionMessage.robotPose.y);
    deltaTheta.z = deltaAngle(currentYaw,yaw_prev);  // delta yaw measured by imu

    POINT_XYZ midTheta;
    midTheta.x = solutionMessage.robotPose.x + 0.5 * deltaTheta.x;
    midTheta.y = solutionMessage.robotPose.y + 0.5 * deltaTheta.y;
    midTheta.z = solutionMessage.robotPose.z + 0.5 * deltaTheta.z;

    POINT_XYZ deltaIndex;
    deltaIndex.x = cos(midTheta.x) * cos(midTheta.z) + sin(midTheta.x) * sin(midTheta.y) * sin(midTheta.z);
    deltaIndex.y = cos(midTheta.x) * sin(midTheta.z) - sin(midTheta.x) * sin(midTheta.y) * cos(midTheta.z);
    deltaIndex.z = sin(midTheta.x) * cos(midTheta.y);

    /***********************************************  evaluate robotDR by IMU and mileage *******************************************/ 
    double deltaMileage = currentMileage - mileage_prev;

    solutionMessage.robotDR.x = solutionMessage.robotDR.x + deltaMileage * deltaIndex.x;
    solutionMessage.robotDR.y = solutionMessage.robotDR.y + deltaMileage * deltaIndex.y;
    solutionMessage.robotDR.z = solutionMessage.robotDR.z + deltaMileage * deltaIndex.z;

    solutionMessage.robotPose.x = constrainAngle(solutionMessage.robotPose.x + deltaTheta.x);
    solutionMessage.robotPose.y = constrainAngle(solutionMessage.robotPose.y + deltaTheta.y);
    solutionMessage.robotPose.z = constrainAngle(solutionMessage.robotPose.z + deltaTheta.z);
    /***********************************************************************************************************************************/ 

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;
}

void robotPosePubish()
{
    geometry_msgs::PoseStamped PositionPub;

    static ros::Time current_time;
    current_time=ros::Time::now();

    PositionPub.header.stamp = current_time;
    PositionPub.header.frame_id = "ekfPose";

    PositionPub.pose.position.x = solutionData.robotEKF.x;
    PositionPub.pose.position.y = solutionData.robotEKF.y;
    PositionPub.pose.position.z = solutionData.robotEKF.z;

    PositionPub.pose.orientation.x = solutionData.robotEuler.x;
    PositionPub.pose.orientation.y = solutionData.robotEuler.y;
    PositionPub.pose.orientation.z = solutionData.robotEuler.z; 
    PositionPub.pose.orientation.w = configData.updateGNSS;

    pose_pub.publish(PositionPub);
}

void initialCallback(geometry_msgs::PoseStamped msg_initial)
{
    configData.baseECEF.x = msg_initial.pose.position.x;
    configData.baseECEF.y = msg_initial.pose.position.y;
    configData.baseECEF.z = msg_initial.pose.position.z;
    configData.baseBLH = ECEFtoBLH(configData.baseECEF);

    configData.enableBase = true; // true is available, false is invalid

    solutionData.robotEKF.x = msg_initial.pose.orientation.x;
    solutionData.robotEKF.y = msg_initial.pose.orientation.y;
    solutionData.robotEKF.z = 0;

    solutionData.robotEuler.x = configData.euler.x;
    solutionData.robotEuler.y = configData.euler.y;//z->y
    solutionData.robotEuler.z = msg_initial.pose.orientation.z;

    ROS_INFO("location get initial angle: x = %f, y = %f , z = %f",solutionData.robotEuler.x*180/PI, solutionData.robotEuler.y*180/PI, solutionData.robotEuler.z*180/PI);

    solutionData.gyroBias.x = 0;
    solutionData.gyroBias.y = 0;
    solutionData.gyroBias.z = msg_initial.pose.orientation.w;

    solutionData.robotECEF = configData.gnssECEF;
    solutionData.robotBLH = configData.gnssBLH;
    solutionData.robotENU =  getCoordinatesENU(configData.gnssECEF,configData.baseECEF,configData.baseBLH);

    ROS_INFO("location get initial ENU: x = %f, y = %f , z = %f",solutionData.robotENU.x, solutionData.robotENU.y, solutionData.robotENU.z);
    ROS_INFO("location get initial BLH: x = %f, y = %f , z = %f",solutionData.robotEKF.x, solutionData.robotEKF.y, solutionData.robotEKF.z);

    Eigen::Matrix3d Cbn = angleToMatrix(solutionData.robotEuler);
    Eigen::Vector3d velocityB(0, 0.5*(configData.odomVel.x + configData.odomVel.y), 0);
    Eigen::Vector3d velocityN = Cbn * velocityB;

    solutionData.robotVel.x = velocityN(0);
    solutionData.robotVel.y = velocityN(1);
    solutionData.robotVel.z = velocityN(2);

    ROS_INFO("location get initial Vel: x = %f, y = %f , z = %f",solutionData.robotVel.x, solutionData.robotVel.y, solutionData.robotVel.z);

    /*********************************************************************************************/

    solutionData.robotDR = solutionData.robotEKF;

    solutionData.robotPose.x = solutionData.robotEuler.x;
    solutionData.robotPose.y = solutionData.robotEuler.y;
    solutionData.robotPose.z = constrainAngle(solutionData.robotEuler.z + PI/2);

    solutionData.initialState = true;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 1
    fout_point << configData.baseECEF.x << "   " << configData.baseECEF.y << "   " << configData.baseECEF.z << "   ";
    fout_point << configData.baseBLH.x << "   " << configData.baseBLH.y << "   " << configData.baseBLH.z << "   "; 
    fout_point << solutionData.robotEuler.z << "   " << solutionData.gyroBias.z  << "   " <<  " 1 "  << std::endl;
}

void odomCallback(const geometry_msgs::PoseArrayConstPtr& msg_odom)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    detectData.update = true;

    detectData.leftSide = (((int) msg_odom->poses[0].position.x) == 1)?true:false;
    detectData.rightSide = (((int) msg_odom->poses[0].position.y) == 1)?true:false;
    detectData.signalSide = (((int) msg_odom->poses[0].position.z) == 1)?true:false;

    detectData.leftValue = msg_odom->poses[1].position.x;
    detectData.rightValue = msg_odom->poses[1].position.y;

    detectData.leftBump = (((int) msg_odom->poses[0].orientation.x) == 1)?true:false;
    detectData.rightBump = (((int) msg_odom->poses[0].orientation.y) == 1)?true:false;
    detectData.leftUp = (((int) msg_odom->poses[0].orientation.z) == 1)?true:false;
    detectData.rightUp = (((int) msg_odom->poses[0].orientation.w) == 1)?true:false;

    /*********************************************************************************/

    configData.dtOdom = dt;
    configData.odomMil.x = msg_odom->poses[1].orientation.x; // left wheel mileage
    configData.odomMil.y = msg_odom->poses[1].orientation.y; // right wheel mileage
    configData.odomMil.z = 0.5*(configData.odomMil.x + configData.odomMil.y);

    configData.odomVel.x = msg_odom->poses[1].orientation.z;  // left wheel velocity
    configData.odomVel.y = msg_odom->poses[1].orientation.w; // right wheel velocity
    configData.odomVel.z = 0.5*(configData.odomVel.x + configData.odomVel.y);

    configData.updateIMU = true;
    /**********************************************************************************************/

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 2
    fout_point << msg_odom->poses[0].position.x << "    " << msg_odom->poses[0].position.y  << "   " <<  msg_odom->poses[0].position.z  << "   ";
    fout_point << msg_odom->poses[0].orientation.x << "   " << msg_odom->poses[0].orientation.y << "   " << msg_odom->poses[0].orientation.z << "   ";
    fout_point << msg_odom->poses[0].orientation.w << "    " << dt  << "   " <<  " 2 " << std::endl;

    // line 3
    fout_point << msg_odom->poses[1].position.x << "    " << msg_odom->poses[1].position.y  << "   " <<  msg_odom->poses[1].position.z  << "   ";
    fout_point << msg_odom->poses[1].orientation.x << "   " << msg_odom->poses[1].orientation.y << "   " << msg_odom->poses[1].orientation.z << "   ";
    fout_point << msg_odom->poses[1].orientation.w << "    " << dt  << "   " <<  " 3 " << std::endl;
}

void imuCallback(const geometry_msgs::PoseArrayConstPtr& msg_imu)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    /**********************************************************************************************/
    configData.dtIMU = dt;

    configData.euler.x = DegreeToRad * msg_imu->poses[0].position.x; // euler angle measured by IMU module
    configData.euler.y = DegreeToRad * msg_imu->poses[0].position.y; 
    configData.euler.z = DegreeToRad * msg_imu->poses[0].position.z;

    configData.accel.x = earth_g * msg_imu->poses[1].position.x;  // acceleration data in body coordinate frame
    configData.accel.y = earth_g * msg_imu->poses[1].position.y;
    configData.accel.z = earth_g * msg_imu->poses[1].position.z;

    configData.gyro.x = DegreeToRad * msg_imu->poses[1].orientation.x; // gyro data velocity in body coordinate frame
    configData.gyro.y = DegreeToRad * msg_imu->poses[1].orientation.y;
    configData.gyro.z = DegreeToRad * msg_imu->poses[1].orientation.z;

    configData.imuTemperature = msg_imu->poses[1].orientation.w; 

    // line 11
    fout_point << msg_imu->poses[0].position.x << "    " << msg_imu->poses[0].position.y  << "   " <<  msg_imu->poses[0].position.z  << "   ";
    fout_point << msg_imu->poses[0].orientation.x << "   " << msg_imu->poses[0].orientation.y << "   " << msg_imu->poses[0].orientation.z << "   ";
    fout_point << msg_imu->poses[0].orientation.w << "    " << dt  << "   " <<  " 11 " << std::endl;

    // line 12
    fout_point << msg_imu->poses[1].position.x << "    " << msg_imu->poses[1].position.y  << "   " <<  msg_imu->poses[1].position.z  << "   ";
    fout_point << msg_imu->poses[1].orientation.x << "   " << msg_imu->poses[1].orientation.y << "   " << msg_imu->poses[1].orientation.z << "   ";
    fout_point << msg_imu->poses[1].orientation.w << "    " << dt  << "   " <<  " 12 " << std::endl;
}

void gnssMessageCallback(const geometry_msgs::PoseStampedConstPtr& msg_position,
                         const geometry_msgs::PoseStampedConstPtr& msg_state)
{
    static ros::Time prev_time = ros::Time::now();
    double dt = ros::Time::now().toSec() - prev_time.toSec();
    prev_time = ros::Time::now();

    configData.dtGNSS = dt;

    configData.enableGNSS = ((int)(msg_position->pose.position.x) == '4')?true:false;  // true is available, false is invalid
    
    configData.gnssBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    configData.gnssBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    configData.gnssBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;
    
    configData.gnssECEF = BLHtoECEF(configData.gnssBLH);
    
    if( configData.enableBase ) configData.gnssENU = getCoordinatesENU(configData.gnssECEF,configData.baseECEF,configData.baseBLH);
                           else configData.gnssENU = configData.baseENU;

    configData.updateGNSS = ((int)(msg_state->pose.orientation.y) == 'A')?true:false;  // true is available, false is invalid

    // ROS_INFO("get rtk ENU: x = %f, y = %f , z = %f",configData.gnssENU.x,configData.gnssENU.y,configData.gnssENU.z);

    double speedGround = 0.5144444 * msg_state->pose.orientation.w;
    double directionGround = constrainAngle(PI/2 - msg_state->pose.orientation.z * DegreeToRad);

    configData.gnssVel.x = speedGround * cos(directionGround);
    configData.gnssVel.y = speedGround * sin(directionGround);
    configData.gnssVel.z = 0;

    /**********************************************************************************************/
    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 4
    fout_point << msg_position->pose.position.x << "   " << msg_position->pose.position.y << "   " << msg_position->pose.position.z <<"   ";
    fout_point << msg_position->pose.orientation.x << "   " << msg_position->pose.orientation.y << "   " << msg_position->pose.orientation.z <<"   ";
    fout_point << msg_position->pose.orientation.w << "   " << dt << "   " << " 4 " << std::endl;

    // line 5
    fout_point << msg_state->pose.position.x << "   " << msg_state->pose.position.y << "   " << msg_state->pose.position.z <<"   ";
    fout_point << msg_state->pose.orientation.x << "   " << msg_state->pose.orientation.y << "   " << msg_state->pose.orientation.z <<"   ";
    fout_point << msg_state->pose.orientation.w << "   " << dt << "   " << " 5 " << std::endl;
    /**********************************************************************************************/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_sensor");
    ROS_INFO_STREAM("start to read and send sensor information ...");

    ros::NodeHandle ns;

    //Publisher
    pose_pub = ns.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);

    //Subcriber
    ros::Subscriber initial_sub = ns.subscribe("/initial_pose", 1, initialCallback);
    ros::Subscriber pose_sub  = ns.subscribe("/alf001_dis", 1,odomCallback);
    ros::Subscriber imu_sub = ns.subscribe("/imu_data", 1, imuCallback);

    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkPosition_sub(ns,"gnss_position",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkState_sub(ns,"gnss_state",1);
    message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10), rtkPosition_sub,rtkState_sub);
    sync.registerCallback(boost::bind(&gnssMessageCallback, _1, _2));

    // ros::Rate loop_rate(10);
    ros::Time prev_whileloop = ros::Time::now();

    initialParameter(configData,solutionData);

    while(ros::ok())
    {
        if(solutionData.initialState)
        {
            // update attitude Kalman solution
            attitudeFilterSolution(configData,solutionData);

            // update the position and velocity Kalman filter solution
            positionFilterSolution(configData,solutionData);

            // update the dead reckoning solution
            deadReckoningSolution(configData,solutionData);

            robotPosePubish();
        }
        ros::spinOnce();
        // loop_rate.sleep();
        ros::Duration(0.1).sleep(); // sleep for 0.1 second
    }

    fout_point.close();
    return 0;
}

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}
