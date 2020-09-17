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
#include "std_msgs/UInt8.h"

//c++
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>

//spdlog
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// map
#include <nav_msgs/OccupancyGrid.h>

// geometry function
#include  "navigateFunction.cpp"
#include  "slamFunction.cpp"

#include "astar.cpp"

using namespace std;

std::string getDate()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
    return tmp;
}

/************************************************************************************/
std::string point_path = "/home/rasp008/mower_ws/src/circuit_zigzag/data/";
// std::string point_path = "/home/yat/catkin_ws/src/circuit_zigzag/data/";
std::string file_log = point_path + getDate() + "motion2.txt";
std::ofstream fout_point(file_log.c_str());

nav_msgs::OccupancyGrid gridMap;

nav_msgs::OccupancyGrid sourceMap;


bool refreshPID = false;
float W_threshold = 0.5;
float V_threshold = 0.35;

poseType robotPose;
poseType startPoint;
YAT_POINT originXY;

geometry_msgs::Twist msg_vel;
ros::Publisher vel_pub;

typedef struct
{
	int num;   // the current node number
    YAT_POINT *node; // record the current node
    int step;
}NodeListType;

typedef struct
{
	bool passed;   // the current node number
    YAT_POINT node; // record the father node
}LastNodeType;

typedef struct
{
	int num;   // cut zone number
    int type;  // 0 : is finish; 1 : along with set direction; 2 : along with vertically set direction;
    int side;  // 0 : go to next zone; 1 : in set direction rightside; 2 : in set direction lefttside;
    bool arrive; // is or not arrive at current zone
    double yaw; // yaw of cut robot follow
    double alongAngle; // the last along yaw 

    int mapNum;
}CutZoneType;

typedef struct
{
    bool updateMap;
    bool updatePose;
    
    bool updateBump;
    bool updateStuck;
    bool updateEdge;

    bool updateAlong;

    bool bumped;
    bool stuck;

    bool stopTask;
    bool finishTask;

    double cutDirection;
    double cutRange;

    int cutZone;

    bool gobackCharging;

    YAT_POINTF originPoint;  // zero point for cut zone
    YAT_POINTF begainPoint;  // zero point for side in cut zone

    YAT_POINTF pathPoint[5];

    ros::Time updateMapTime;
    ros::Time updatePoseTime;
    ros::Time randomStart;
}SENSOR_MESSAGE;

ODOM_TYPE odomData;
DETECT_TYPE detectData;
SENSOR_MESSAGE sensorData;

void initializationMession(SENSOR_MESSAGE &sensorMsg)
{
    sensorMsg.updateMap = false;
    sensorMsg.updatePose = false;
    
    sensorMsg.updateBump = false;
    sensorMsg.updateStuck = false;
    sensorMsg.updateEdge = false;

    sensorMsg.updateAlong = true;

    sensorMsg.bumped = false;
    sensorMsg.stuck = false;

    sensorMsg.stopTask = false;
    sensorMsg.finishTask = false;

    sensorMsg.cutDirection = 0;
    sensorMsg.cutRange = 6;
    sensorMsg.cutZone = 0;

    sensorMsg.gobackCharging = false;

    sensorMsg.updateMapTime = ros::Time::now();
    sensorMsg.updatePoseTime = ros::Time::now();
}

void initializationSensor(SENSOR_MESSAGE &sensorMsg)
{
    sensorMsg.updateMap = false;
    sensorMsg.updatePose = false;
    
    sensorMsg.updateBump = false;
    sensorMsg.updateStuck = false;
    sensorMsg.updateEdge = false;
}

bool chargeDetect(bool &isCharging)
{
    bool finishTask = false;
    static int filterTimes = 0;

    if(filterTimes > 3)
    {
        if(detectData.chargeCurrent > 200)
        {
            if(filterTimes > 5) 
            {
                isCharging = true;
                finishTask = true;
                filterTimes = 0;
            }
        }
        else
        {
            finishTask = true;
            isCharging = false;
            filterTimes = 0;
        } 
    }

    filterTimes++;

    return finishTask;
}

/********************************  motion control  *******************************/
YAT_POINTF move_forward(YAT_POINTF targetPoint, YAT_POINTF lastPoint)
{   
    // angle control
    float pathDistance = deltaDistance(targetPoint , lastPoint); 

    YAT_POINTF pathVector;
    pathVector.x = (targetPoint.x - lastPoint.x) / pathDistance;
    pathVector.y = (targetPoint.y - lastPoint.y) / pathDistance;

    YAT_POINTF targetVector;
    targetVector.x = targetPoint.x - robotPose.x;
    targetVector.y = targetPoint.y - robotPose.y;

    float trajectoryDistance = distanceToAB(targetPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double deadLineDistance = targetVector.x * pathVector.x + targetVector.y * pathVector.y;

    float aimLength = 0.5; // units: m
 
    float controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength )
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    float pathAngle = atan2( targetPoint.y - lastPoint.y, targetPoint.x - lastPoint.x);
    float targetAngle = constrainAngle(controlAngle + pathAngle);
    float controlObject = deltaAngle(targetAngle,robotPose.theta);  // - odomData.pose.orientation.z;

    YAT_POINTF commandVelocity;

    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

    // line velocity control, limit velocity
    if( fabs(commandVelocity.x) > V_threshold)
    {
         commandVelocity.x = V_threshold * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * sign(commandVelocity.x);
    }

    commandVelocity.y = controlObject * 2.4 / PI;

    // angle rate control, limit angle velocity
    if (fabs(commandVelocity.y) > W_threshold) commandVelocity.y = V_threshold * sign( commandVelocity.y );


    return commandVelocity;
}

YAT_POINTF move_back(YAT_POINTF targetPoint, YAT_POINTF lastPoint)
{
    // angle control
    float pathDistance = deltaDistance(targetPoint , lastPoint); 

    YAT_POINTF pathVector;
    pathVector.x = (targetPoint.x - lastPoint.x) / pathDistance;
    pathVector.y = (targetPoint.y - lastPoint.y) / pathDistance;

    YAT_POINTF targetVector;
    targetVector.x = targetPoint.x - robotPose.x;
    targetVector.y = targetPoint.y - robotPose.y;

    float trajectoryDistance = distanceToAB(targetPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double deadLineDistance = targetVector.x * pathVector.x + targetVector.y * pathVector.y;

    float aimLength = 0.5; // units: m
 
    float controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength )
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    float pathAngle = atan2( targetPoint.y - lastPoint.y, targetPoint.x - lastPoint.x);
    float targetAngle = constrainAngle(controlAngle + pathAngle);
    float controlObject = deltaAngle(targetAngle,robotPose.theta);  // - odomData.pose.orientation.z;

    YAT_POINTF commandVelocity;

    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

    // line velocity control, limit velocity
    if( fabs(commandVelocity.x) > V_threshold)
    {
         commandVelocity.x = V_threshold * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.1)
    {
         commandVelocity.x = 0.1 * sign(commandVelocity.x);
    }

    // angle rate control, limit angle velocity
    commandVelocity.y = 0;

    return commandVelocity;
}

YAT_POINTF turning(YAT_POINTF targetPoint, YAT_POINTF lastPoint)
{
    float targetAngle = atan2(targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x);
    float  deltaTheta = deltaAngle(targetAngle, robotPose.theta);

    YAT_POINTF commandVelocity;

    commandVelocity.x = 0;
    commandVelocity.y = deltaTheta * 1.8 / PI;

    if(fabs(commandVelocity.y) < 0.2)
    {
        commandVelocity.y = 0.2 * sign(deltaTheta);
    }
    else if(fabs(commandVelocity.y) > 0.5)
    {
        commandVelocity.y = 0.5 * sign(deltaTheta);
    }

    return commandVelocity;
}

bool turning(double targetAngle,YAT_POINTF &commandVelocity)
{
    float  deltaTheta = deltaAngle(targetAngle, robotPose.theta);

    commandVelocity.x = 0;
    commandVelocity.y = 0.35 * sign(deltaTheta);

    if(fabs(deltaTheta) < 5 * DegreeToRad)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;
        return true;
    }
    else
    {
        return false;
    }
}

bool waitTimes(int waitNum)
{
    static int num = 0;
    
    num++;
    
    if(num >= waitNum)
    {
        num=0;
        return true;
    }
    else
    {
        return false;
    } 
}

bool turnedTarget(YAT_POINTF targetPoint, YAT_POINTF lastPoint,float turnDoneAngle)
{
    float targetAngle = atan2( targetPoint.y - lastPoint.y , targetPoint.x - lastPoint.x );
    bool turnedFlag = ( fabs( deltaAngle( targetAngle , robotPose.theta ) ) < degreeToRad(turnDoneAngle) ) ?true:false;

    return turnedFlag;
}

bool arrivedTarget(YAT_POINTF targetPoint, YAT_POINTF lastPoint,float moveDoneLength)
{
    Eigen::Vector2d pathVector(0,0);
    Eigen::Vector2d targetVector(0,0);
    
    float pathDistance = deltaDistance( targetPoint, lastPoint );

    pathVector(0) = (targetPoint.x - lastPoint.x) / pathDistance;
    pathVector(1) = (targetPoint.y - lastPoint.y) / pathDistance;
    
    targetVector(0) = targetPoint.x - robotPose.x;
    targetVector(1) = targetPoint.y - robotPose.y;

    float deadDistance = ( pathVector.transpose() )*targetVector;

    bool arrivedFlag = ( deadDistance < moveDoneLength )?true:false;

    return arrivedFlag;
}

bool moveAlongBoundary(bool alongWise,YAT_POINTF &commandVelocity,bool &turningFlag)
{
    static int motionStep = 0;
    bool motionFinish = false;

    static float totalAngle = 0;
    static float lastAngle = robotPose.theta;

    static YAT_POINTF pointB;
    static YAT_POINTF pointA;

/***************************************************************************************/

    bool alongEdge = false; // along the doundary
    bool reverseEdge = false; // reverse the doundary

    bool inRightSide = false; // in the rightside of boundary
    bool inLeftSide = false; // in the leftside of boundary
    
    if(alongWise)
    {
        // clock wise
        alongEdge = detectData.rightSide && (!detectData.leftSide);
        reverseEdge = (!detectData.rightSide) && detectData.leftSide;

        inRightSide = detectData.rightSide && detectData.leftSide;
        inLeftSide = (!detectData.rightSide) && (!detectData.leftSide);
    }
    else
    {
        // anti-clock wise
        alongEdge = (!detectData.rightSide) && detectData.leftSide;
        reverseEdge = detectData.rightSide && (!detectData.leftSide);

        inLeftSide = detectData.rightSide && detectData.leftSide;
        inRightSide = (!detectData.rightSide) && (!detectData.leftSide);
    }

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;
/***************************************************************************************/

    if(sensorData.updateAlong)
    {
        motionStep = 0;
        sensorData.updateAlong = false;
    }

    if(motionStep == 0)
    {
        lastAngle = robotPose.theta;
        totalAngle = 0;

        pointB.x = robotPose.x + 0.15 * cos(robotPose.theta);
        pointB.y = robotPose.y + 0.15 * sin(robotPose.theta);

        pointA.x = robotPose.x;
        pointA.y = robotPose.y;

        motionStep = 1;
    }

    // find the boundary
    if(motionStep == 1)
    {
        double targetAngle = robotPose.theta;
        if(inRightSide)
        {
            targetAngle = constrainAngle(robotPose.theta + PI/2);
        }
        else if(inLeftSide)
        {
            targetAngle = constrainAngle(robotPose.theta - PI/2);
        }
        else if(reverseEdge)
        {
            targetAngle = constrainAngle(robotPose.theta + PI/2);
        }

        turning(targetAngle,commandVelocity);
 
        totalAngle = totalAngle + fabs(constrainAngle(robotPose.theta - lastAngle));
        lastAngle = robotPose.theta;

        if(fabs(totalAngle) > 2 * PI)
        {
            motionStep = 8;
            ROS_INFO("Arround a circle, did not find the boundary, mission is finish.");
        }

        if(alongEdge)
        {
            totalAngle = 0;
            motionStep = 2;
        }
    }

    // along the boundary
    if(motionStep == 2)
    {
        if(inRightSide) // in the right side
        {
            double errorAngle = atan2(0.08132,0.34);

            pointB.x = robotPose.x + 0.15 * cos(robotPose.theta + errorAngle);
            pointB.y = robotPose.y + 0.15 * sin(robotPose.theta + errorAngle);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 3;
        }
        else if(inLeftSide) // in the left side
        {
            double errorAngle = atan2(0.08132,0.34);

            pointB.x = robotPose.x + 0.15 * cos(robotPose.theta - errorAngle);
            pointB.y = robotPose.y + 0.15 * sin(robotPose.theta - errorAngle);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 3;
        }
        else if(alongEdge) // in the boundary
        {
            double deltaL = 0.0832 * detectData.leftValue / (detectData.leftValue + detectData.rightValue);
            double deltaR = 0.0832 * detectData.rightValue / (detectData.leftValue + detectData.rightValue);

            // double errorAngle = atan2(deltaR - deltaL,0.34);
            double errorAngle = (alongWise == true) ?(atan2(deltaR - deltaL,0.34)):(atan2(deltaL - deltaR,0.34));

            pointB.x = robotPose.x + 0.3 * cos(robotPose.theta + errorAngle);
            pointB.y = robotPose.y + 0.3 * sin(robotPose.theta + errorAngle);

            // ROS_INFO("in side -> deltaR : %f, deltaL: %f.",deltaR,deltaL);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;
        }
        
        commandVelocity = move_forward(pointB,pointA);

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump, finish the mission.");
            motionStep = 6; // motionStep = 6;

            pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
            pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;
        }
    }

    if(motionStep == 3)
    {
        commandVelocity = move_forward(pointB,pointA);

        if(arrivedTarget(pointB,pointA,0.05))
        {
            motionStep = 4;
        }

        if(alongEdge) motionStep = 2;
    }

    if(motionStep == 4)
    {
        if(robotStop)
        {
            motionStep = 5;

            totalAngle = 0;
            lastAngle = robotPose.theta;
        }

        commandVelocity.x = 0;
        commandVelocity.y = 0;
    }

    if(motionStep == 5)
    {
        double targetAngle = robotPose.theta;
        if(inRightSide)
        {
            targetAngle = constrainAngle(robotPose.theta + PI/2);
        }
        else if(inLeftSide)
        {
            targetAngle = constrainAngle(robotPose.theta - PI/2);
        }
        else if(reverseEdge)
        {
            targetAngle = constrainAngle(robotPose.theta + PI/2);
        }

        turning(targetAngle,commandVelocity);

        if(detectData.rightSide && detectData.leftSide) turningFlag = true;

        totalAngle = totalAngle + constrainAngle(robotPose.theta - lastAngle);
        lastAngle = robotPose.theta;

        if(fabs(totalAngle) > 2 * PI )
        {
            motionStep = 8;

            ROS_ERROR("Arround a circle, did not get the enable angle,finish the mission.");
        }

        if(alongEdge)
        {
            totalAngle = 0;
            motionStep = 2;
        }
    }

    if(motionStep == 6)
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB,pointA, 0.05))
        {
            motionStep = 8;
        }
    }

    if(motionStep == 8)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 0;
            totalAngle = 0;

            motionFinish = true;
        }
    }

    spdlog::get("robot_state")->info("move along boundary : the step is {}.",motionStep);

    return motionFinish;
}

/************************************************************************************/
gridMapType getGridParametre(nav_msgs::OccupancyGrid data)
{
    gridMapType parametre;

    parametre.scale = (int)(1/(data.info.resolution));
    parametre.rangeX = data.info.width;
    parametre.rangeY = data.info.height;
    parametre.offsetX = data.info.origin.position.x;
    parametre.offsetY = data.info.origin.position.y;

    return parametre;
}


double distanceCenterLine(CutZoneType *zoneMsg,YAT_POINTF testPoint)
{
    YAT_POINTF vectoryYaw;
    vectoryYaw.x = cos(zoneMsg->yaw + PI/2);
    vectoryYaw.y = sin(zoneMsg->yaw + PI/2);

    YAT_POINTF zoneOrigin;
    zoneOrigin.x = sensorData.originPoint.x + zoneMsg->num * 5.0 * vectoryYaw.x;
    zoneOrigin.y = sensorData.originPoint.y + zoneMsg->num * 5.0 * vectoryYaw.y;

    return (testPoint.x - zoneOrigin.x) * vectoryYaw.x + (testPoint.y - zoneOrigin.y) * vectoryYaw.y;
}

YAT_POINTF worldPointToMap(CutZoneType *zoneMsg,YAT_POINTF testPoint)
{
    YAT_POINTF vectoryYaw;
    vectoryYaw.x = cos(zoneMsg->yaw + PI/2);
    vectoryYaw.y = sin(zoneMsg->yaw + PI/2);

    YAT_POINTF zoneOrigin;
    zoneOrigin.x = sensorData.originPoint.x + zoneMsg->mapNum * 5.0 * vectoryYaw.x;
    zoneOrigin.y = sensorData.originPoint.y + zoneMsg->mapNum * 5.0 * vectoryYaw.y;

    YAT_POINTF deltaPosition;
    deltaPosition.x = testPoint.x - zoneOrigin.x;
    deltaPosition.y = testPoint.y - zoneOrigin.y;

    YAT_POINTF relatedPosition;
    relatedPosition.x = cos(zoneMsg->yaw) * deltaPosition.x + sin(zoneMsg->yaw) * deltaPosition.y;
    relatedPosition.y = - sin(zoneMsg->yaw) * deltaPosition.x + cos(zoneMsg->yaw) * deltaPosition.y;

    return relatedPosition;
}

YAT_POINTF mapPointToWorld(CutZoneType *zoneMsg,YAT_POINTF testPoint)
{
    YAT_POINTF deltaPosition;
    deltaPosition.x = cos(zoneMsg->yaw) * testPoint.x - sin(zoneMsg->yaw) * testPoint.y;
    deltaPosition.y = sin(zoneMsg->yaw) * testPoint.x + cos(zoneMsg->yaw) * testPoint.y;

    YAT_POINTF vectoryYaw;
    vectoryYaw.x = cos(zoneMsg->yaw + PI/2);
    vectoryYaw.y = sin(zoneMsg->yaw + PI/2);

    YAT_POINTF zoneOrigin;
    zoneOrigin.x = sensorData.originPoint.x + zoneMsg->mapNum * 5.0 * vectoryYaw.x;
    zoneOrigin.y = sensorData.originPoint.y + zoneMsg->mapNum * 5.0 * vectoryYaw.y;

    YAT_POINTF relatedPosition;
    relatedPosition.x = deltaPosition.x + zoneOrigin.x;
    relatedPosition.y = deltaPosition.y + zoneOrigin.y;

    return relatedPosition;
}

void initialGridMap(MAP_GRID *zoneMap)
{   
    ROS_INFO("initial the zone map : flag 0.");

    zoneMap->scale = 10;

    zoneMap->rangeX = 400;
    zoneMap->rangeY = 80;

    zoneMap->offsetX = - 20;
    zoneMap->offsetY = - 4;

    ROS_INFO("initial the zone map : flag 1.");

    zoneMap->data = (int *)malloc(zoneMap->rangeX * zoneMap->rangeY * sizeof(int));

    ROS_INFO("initial the zone map : flag 2.");

    for(int j = 0; j < zoneMap->rangeY; j++)
    {
        for(int i = 0; i < zoneMap->rangeX; i++)
        {
            zoneMap->data[j * zoneMap->rangeX + i] = ID_REMAIN;
        }
    }

    ROS_INFO("finish initial the zone map.");
}

void findMaximumVaule(MAP_GRID *zoneMap,YAT_POINT &maxPoint,YAT_POINT &minPoint)
{
    YAT_POINT maxXY;
    maxXY.x = 0;
    maxXY.y = 0;

    YAT_POINT minXY;
    minXY.x = zoneMap->rangeX;
    minXY.y = zoneMap->rangeY;

    for(int j = 0; j < zoneMap->rangeY; j++)
    {
        for(int i = 0; i < zoneMap->rangeX; i++)
        {
            bool isBoundary = (zoneMap->data[j * zoneMap->rangeX + i] == ID_OBSTACLE)?true:false;
            bool isCut = (zoneMap->data[j * zoneMap->rangeX + i] == ID_CUT)?true:false;
            if(isBoundary || isCut)
            {
                if(i < minXY.x) minXY.x = i;
                if(i > maxXY.x) maxXY.x = i;

                if(j < minXY.y) minXY.y = j;
                if(j > maxXY.y) maxXY.y = j;
            }
        }
    }

    maxPoint = maxXY;
    minPoint = minXY;

    spdlog::get("robot_state")->info("map range X : {} -> {}.",minXY.x,maxXY.x);
    spdlog::get("robot_state")->info("map range Y : {} -> {}.",minXY.y,maxXY.y);
}

void dilateMap(MAP_GRID *mapGrid, double dilatewidth)
{
	bool *mask = (bool*)malloc(sizeof(bool)*(mapGrid->rangeX * mapGrid->rangeY));
	memset(mask, false, mapGrid->rangeX * mapGrid->rangeY);

	int dilate = (int)(dilatewidth * mapGrid->scale);

	if (dilate < 2)
	{
		dilate = 2;
	}

	for (int i = 0; i < mapGrid->rangeY; i++)
	{
		for (int j = 0; j < mapGrid->rangeX; j++)
		{
			bool flag = false;
			for (int k = -dilate; k <= dilate; k++)
			{
				if ((i + k) < 0 || (i + k) >= mapGrid->rangeY) continue;
				for (int l = -dilate; l <= dilate; l++)
				{
					if ((j + l) < 0 || (j + l) >= mapGrid->rangeX) continue;
					if (*(mapGrid->data + (i + k) * mapGrid->rangeX + j + l) == ID_OBSTACLE) mask[i * mapGrid->rangeX + j] = true;
				}
			}
		}
	}

	for (int i = 0; i < mapGrid->rangeY; i++)
	{
		for (int j = 0; j < mapGrid->rangeX; j++)
		{
			if (mask[i * mapGrid->rangeX + j] == true )	mapGrid->data[i * mapGrid->rangeX + j] = ID_OBSTACLE;
		}
	}

	free(mask);
}

YAT_POINT findNestEnablePoint(MAP_GRID *zoneMap,YAT_POINT detectPoint)
{
    YAT_POINT nestPoint = detectPoint;
    int minStep = fabs(zoneMap->rangeX) + fabs(zoneMap->rangeY);

    gridMapType mapParam = getGridParametre(zoneMap);

    for(int j = 0; j < zoneMap->rangeY; j++)
    {
        for(int i = 0; i < zoneMap->rangeX; i++)
        {
            bool isCut = true;
            (zoneMap->data[j * zoneMap->rangeX + i] == ID_CUT)?true:false;

            for(int m = -1; m <= 1; m++)
            {
                for(int n = -1; n <= 1; n++)
                {
                    YAT_POINT testXY;
                    testXY.x = i + m;
                    testXY.y = j + n;

                    if(detectPointXY(mapParam,testXY))
                    {
                        if(zoneMap->data[testXY.y * zoneMap->rangeX + testXY.x] != ID_CUT) isCut = false;
                    }
                    else
                    {
                        isCut = false;
                    }
                    if(!isCut) break;
                }
                if(!isCut) break;
            }

            int moveStep = fabs(detectPoint.x - i) + fabs(detectPoint.y - j); 
            bool isMinStep = (moveStep < minStep)?true:false;
            if(isCut && isMinStep)
            {
                nestPoint.x = i;
                nestPoint.y = j;
                minStep = moveStep;
            }
        }
    }

    return nestPoint;
}

bool findLeftRemain(MAP_GRID *mapGrid,CutZoneType *zoneMsg, YAT_POINTF &targetPoint)
{
    gridMapType parametre = getGridParametre(mapGrid);

    bool findRemain = false;
    bool findNode = false;

    YAT_POINT remainPoint[100];
    int remainNum = 0;

    for(int j = 10; j < 70; j++)
    {
        findRemain = false;
        for(int i = 0; i < mapGrid->rangeX; i++)
        {
            int pointData = mapGrid->data[j * mapGrid->rangeX + i];
            if(pointData == ID_CUT)
            {
                remainPoint[0].x = i;
                remainPoint[0].y = j;

                YAT_POINTF remainPosition = rasterToWorld(remainPoint[0],parametre);
                targetPoint = mapPointToWorld(zoneMsg,remainPosition);

                findRemain = true;
                break;
            }
            else if(pointData == ID_REMAIN)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        if(findRemain) break;
    }

    return findRemain;
}

bool findRightRemain(MAP_GRID *mapGrid,CutZoneType *zoneMsg, YAT_POINTF &targetPoint)
{
    gridMapType parametre = getGridParametre(mapGrid);

    bool findRemain = false;
    bool findNode = false;

    YAT_POINT remainPoint[100];
    int remainNum = 0;

    for(int j = 10; j < 70; j++)
    {
        findRemain = false;
        for(int i = mapGrid->rangeX - 1; i > 0; i--)
        {
            int pointData = mapGrid->data[j * mapGrid->rangeX + i];
            if(pointData == ID_CUT)
            {
                remainPoint[0].x = i;
                remainPoint[0].y = j;

                YAT_POINTF remainPosition = rasterToWorld(remainPoint[0],parametre);
                targetPoint = mapPointToWorld(zoneMsg,remainPosition);

                findRemain = true;
                break;
            }
            else if(pointData == ID_REMAIN)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        if(findRemain) break;
    }

    return findRemain;
}

bool findRemainZone(MAP_GRID *zoneMap,CutZoneType *zoneMsg, YAT_POINTF &targetPoint)
{
    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINTF zoneOrigin = worldPointToMap(zoneMsg,sensorData.begainPoint);

    MAP_GRID *mapGrid = (MAP_GRID *)malloc(sizeof(MAP_GRID));

    mapGrid->scale = zoneMap->scale;
    mapGrid->rangeX = zoneMap->rangeX;
    mapGrid->rangeY = zoneMap->rangeY;
    mapGrid->offsetX = zoneMap->offsetX;
    mapGrid->offsetY = zoneMap->offsetY;

    mapGrid->data = (int *)malloc(sizeof(int)* (mapGrid->rangeX * mapGrid->rangeY));

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX ; i++)
        {
            mapGrid->data[j * mapGrid->rangeX + i] = zoneMap->data[j * mapGrid->rangeX + i];
        }
    }
    
    dilateMap(mapGrid, 1.0);

    bool findRemain = false;

    switch(zoneMsg->side)
    {
        case 1:
            findRemain = findRightRemain(mapGrid,zoneMsg,targetPoint);
            break;
        case 2:
            findRemain = findLeftRemain(mapGrid,zoneMsg,targetPoint);
            break;
        default:
            ROS_ERROR("find Remain Zone: error zoneMsg->side");
            break;
    }

    free(mapGrid->data);
    free(mapGrid);

    return findRemain;
}

bool findTopPoint(MAP_GRID *zoneMap,CutZoneType *zoneMsg,YAT_POINTF &topPoint)
{
    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINTF zoneOrigin = worldPointToMap(zoneMsg,sensorData.begainPoint);

    bool findTop = false;
    bool findNode = false;

    YAT_POINT remainPoint[500];
    int remainNum = 0;
    YAT_POINT remainXY;

    for(int i = 0; i < zoneMap->rangeX; i++)
    {
        findNode = false;
        findTop = false;
        for(int j = 65; j < zoneMap->rangeY - 1; j++)
        {
            int pointData = zoneMap->data[j * zoneMap->rangeX + i];
            if(pointData == ID_CUT)
            {
                remainXY.x = i;
                remainXY.y = j;
                findNode = true;
                findTop = false;
            }
            else if(pointData == ID_REMAIN)
            {
                if(findNode) findTop = true;
            }
            else
            {
                if(findNode) 
                {
                    findTop = false;
                    break;
                }
            }
        }

        if(findTop)
        {
            remainPoint[remainNum] = remainXY;
            remainNum ++;
        }
        else
        {
            if(remainNum >= 20)
            {
                findTop = true;
                break;
            }
            else
            {
                remainNum = 0;
            }
        } 
    }

    YAT_POINTF relativePosition = zoneOrigin;
    if(findTop)
    {
        YAT_POINTF positionA = rasterToWorld(remainPoint[0],parametre);
        YAT_POINTF positionB = rasterToWorld(remainPoint[remainNum-1],parametre);

        relativePosition.x = 0.5 * (positionA.x + positionB.x);
        relativePosition.y = 0.5 * (positionA.y + positionB.y);

        topPoint = mapPointToWorld(zoneMsg,relativePosition);
    }

    return findTop;
}

bool findBottomPoint(MAP_GRID *zoneMap,CutZoneType *zoneMsg,YAT_POINTF &bottomPoint)
{
    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINTF zoneOrigin = worldPointToMap(zoneMsg,sensorData.begainPoint);

    bool findBottom = false;
    bool findNode = false;

    YAT_POINT remainPoint[500];
    int remainNum = 0;
    YAT_POINT remainXY;

    for(int i = 0; i < zoneMap->rangeX; i++)
    {
        findNode = false;
        findBottom = false;
        for(int j = 15; j >= 0; j--)
        {
            int pointData = zoneMap->data[j * zoneMap->rangeX + i];
            if(pointData == ID_CUT)
            {
                remainXY.x = i;
                remainXY.y = j;
                findNode = true;
                findBottom = false;
            }
            else if(pointData == ID_REMAIN)
            {
                if(findNode) findBottom = true;
            }
            else
            {
                if(findNode) 
                {
                    findBottom = false;
                    break;
                }
            }
        }

        if(findBottom)
        {
            remainPoint[remainNum] = remainXY;
            remainNum ++;
        }
        else
        {
            if(remainNum >= 20)
            {
                findBottom = true;
                break;
            }
            else
            {
                remainNum = 0;
            }
        } 
    }

    YAT_POINTF relativePosition = zoneOrigin;
    if(findBottom)
    {
        YAT_POINTF positionA = rasterToWorld(remainPoint[0],parametre);
        YAT_POINTF positionB = rasterToWorld(remainPoint[remainNum-1],parametre);

        relativePosition.x = 0.5 * (positionA.x + positionB.x);
        relativePosition.y = 0.5 * (positionA.y + positionB.y);

        bottomPoint = mapPointToWorld(zoneMsg,relativePosition);
    }

    return findBottom;
}

bool enableStraightArrive(MAP_GRID *zoneMap,YAT_POINT pathPointA,YAT_POINT pathPointB)
{
    bool enableMove = true;

    gridMapType mapParam = getGridParametre(zoneMap);

    YAT_POINTF startPoint = rasterToWorld(pathPointA,mapParam);
    YAT_POINTF endingPoint = rasterToWorld(pathPointB,mapParam);

    int departNum = (int)(2 * deltaDistance(startPoint,endingPoint) * zoneMap->scale );

    if(departNum < 2)
    {
        departNum = 2;
    }

    for(int i = 0; i < departNum; i++)
    {
        YAT_POINTF pathPoint;
        pathPoint.x = ( (departNum - i) * startPoint.x + i * endingPoint.x) / departNum;
        pathPoint.y = ( (departNum - i) * startPoint.y + i * endingPoint.y) / departNum;
        
        YAT_POINT pathXY = worldToRaster(pathPoint,mapParam);

        if((!detectPointXY(mapParam,pathXY)) || (zoneMap->data[pathXY.y * zoneMap->rangeX + pathXY.x] != ID_CUT))
        {
            enableMove = false;
            break;
        }
    }

    return enableMove;
}

int getAlongBoundaryWise(MAP_GRID *zoneMap,CutZoneType *zoneMsg,YAT_POINTF currentPoint)
{

    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINT maxPoint;
    YAT_POINT minPoint;

    findMaximumVaule(zoneMap,maxPoint,minPoint);

    YAT_POINTF offsetPoint = rasterToWorld(minPoint,parametre);

    MAP_GRID *mapGrid = (MAP_GRID *)malloc(sizeof(MAP_GRID));

    mapGrid->scale = zoneMap->scale;
    mapGrid->rangeX = maxPoint.x - minPoint.x + 1;
    mapGrid->rangeY = maxPoint.y - minPoint.y + 1;
    mapGrid->offsetX = offsetPoint.x;
    mapGrid->offsetY = offsetPoint.y;

    mapGrid->data = (int *)malloc(sizeof(int)* (mapGrid->rangeX * mapGrid->rangeY));

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX ; i++)
        {
            mapGrid->data[j * mapGrid->rangeX + i] = zoneMap->data[(j + minPoint.y)* zoneMap->rangeX + (i + minPoint.x)];
        }
    }
    
    dilateMap(mapGrid, 0.3);

    bool findLeftSide = false;
    bool findRightSide = false;

    int *sideFlag = (int *)malloc(sizeof(int) * (mapGrid->rangeX * mapGrid->rangeY));

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX; i++)
        {
            sideFlag[j * mapGrid->rangeX + i] = 0;
        }
    }

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX; i++)
        {
            if(mapGrid->data[j * mapGrid->rangeX + i] == ID_OBSTACLE)
            {
                if(i  < mapGrid->rangeX - 1)
                {
                    if(mapGrid->data[j * mapGrid->rangeX + i + 1] == ID_CUT)
                    {
                        sideFlag[j * mapGrid->rangeX + i] = -1;
                        findLeftSide = true;
                    }
                }
            }
            if(findLeftSide) break;
        }

        for(int i = mapGrid->rangeX - 1; i >= 0; i--)
        {
            if(mapGrid->data[j * mapGrid->rangeX + i] == ID_OBSTACLE)
            {
                if(i > 0)
                {
                    if(mapGrid->data[j * mapGrid->rangeX + i - 1] == ID_CUT)
                    {
                        sideFlag[j * mapGrid->rangeX + i] = 1;
                        findRightSide = true;
                    }
                }
            }
            if(findRightSide) break;
        }

        findRightSide = false;
        findLeftSide = false;
    }    

    int minDistance = fabs(mapGrid->rangeX) + fabs(mapGrid->rangeY);
    int alongWise = 0;

    gridMapType mapParam = getGridParametre(mapGrid);
    YAT_POINTF startPoint = worldPointToMap(zoneMsg,currentPoint);
    YAT_POINT currentXY = worldToRaster(startPoint,mapParam);

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX; i++)
        {
            bool isMin = (fabs(currentXY.x - i) + fabs(currentXY.y - j) < minDistance)?true:false;
            bool isBoundary = (fabs(sideFlag[j * mapGrid->rangeX + i]) == 1)?true:false;
            if(isMin && isBoundary)
            {
                alongWise = sideFlag[j * mapGrid->rangeX + i];
                minDistance = fabs(currentXY.x - i) + fabs(currentXY.y - j);
            }
        }
    }

    free(mapGrid->data);
    free(mapGrid);
    free(sideFlag);

    return alongWise;
}

bool getRandomPath(MAP_GRID *zoneMap,CutZoneType *zoneMsg,YAT_POINTF targetPoint)
{
    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINT maxPoint;
    YAT_POINT minPoint;

    findMaximumVaule(zoneMap,maxPoint,minPoint);

    YAT_POINTF offsetPoint = rasterToWorld(minPoint,parametre);

    MAP_GRID *mapGrid = (MAP_GRID *)malloc(sizeof(MAP_GRID));

    mapGrid->scale = zoneMap->scale;
    mapGrid->rangeX = maxPoint.x - minPoint.x + 1;
    mapGrid->rangeY = maxPoint.y - minPoint.y + 1;
    mapGrid->offsetX = offsetPoint.x;
    mapGrid->offsetY = offsetPoint.y;

    mapGrid->data = (int *)malloc(sizeof(int)* (mapGrid->rangeX * mapGrid->rangeY));

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX ; i++)
        {
            mapGrid->data[j * mapGrid->rangeX + i] = zoneMap->data[(j + minPoint.y)* zoneMap->rangeX + (i + minPoint.x)];
        }
    }
    
    dilateMap(mapGrid, 0.3);

    spdlog::get("robot_state")->info("get next zone path :  target Point -> x = {} , y = {}.",targetPoint.x,targetPoint.y);

    gridMapType mapParam = getGridParametre(mapGrid);

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    YAT_POINTF startPoint = worldPointToMap(zoneMsg,robotPoint);
    YAT_POINT startXY = worldToRaster(startPoint,mapParam);
    YAT_POINT pathPointA = findNestEnablePoint(mapGrid,startXY);

    YAT_POINTF endingPoint = worldPointToMap(zoneMsg,targetPoint);
    YAT_POINT endingXY = worldToRaster(endingPoint,mapParam);
    YAT_POINT pathPointB = findNestEnablePoint(mapGrid,endingXY);

    if(enableStraightArrive(mapGrid,pathPointA,pathPointB))
    {
        sensorData.pathPoint[0] = targetPoint;

        free(mapGrid->data);
        free(mapGrid);

        spdlog::get("robot_state")->info("get straight path point to next point: x = {}, y = {}.",targetPoint.x,targetPoint.y);

        return true;
    }

    YAT_POINTF *walkability = (YAT_POINTF *)malloc(sizeof(YAT_POINTF)* (mapGrid->rangeX * mapGrid->rangeY));
    int enableNum = 0;

    bool sameFlag = false;
    float minDistance = 100;
    YAT_POINTF samePoint;
    samePoint.x = 0;
    samePoint.y = 0;

    bool findPath = false;

    for(int i = 0; i < mapGrid->rangeX; i++)
    {
        for(int j = 0; j < mapGrid->rangeY; j++)
        {
            if(mapGrid->data[j * mapGrid->rangeX + i] == ID_CUT)
            {
                YAT_POINT currentXY;
                currentXY.x = i;
                currentXY.y = j;

                YAT_POINTF currentPoint = rasterToWorld(currentXY,mapParam);
                bool reachPointA = enableStraightArrive(mapGrid,pathPointA,currentXY);

                if(reachPointA)
                {
                    walkability[enableNum] = currentPoint;
                    enableNum ++;
                }
            }
        }
    }

    if(enableNum >= 10)
    {
        sensorData.pathPoint[0] = mapPointToWorld(zoneMsg,walkability[(rand() % enableNum)]);
        findPath = true;
        spdlog::get("robot_state")->info("get random path point to next point: x = {}, y = {}.",sensorData.pathPoint[0].x,sensorData.pathPoint[0].y);
    }

    free(mapGrid->data);
    free(mapGrid);
    free(walkability);

    return findPath;
}

void initialNodeLst(NodeListType *listA,NodeListType *listB,LastNodeType *moveList,MAP_GRID *zoneMap,YAT_POINT pointA,YAT_POINT pointB,NodeListType *pathNode)
{
    listA->node = (YAT_POINT*)malloc(sizeof(YAT_POINT) * 2 * (zoneMap->rangeX + zoneMap->rangeY) );
    listB->node = (YAT_POINT*)malloc(sizeof(YAT_POINT) * 2 * (zoneMap->rangeX + zoneMap->rangeY) );

    pathNode->node = (YAT_POINT*)malloc(sizeof(YAT_POINT) * zoneMap->rangeX * zoneMap->rangeY );
    pathNode->num = 0;

    listA->num = 1;
    listA->step = 0;
    listA->node[0] = pointA;

    listB->num = 1;
    listA->step = 0;
    listB->node[0] = pointB;

    for(int i = 0; i < zoneMap->rangeY; i++)
    {
        for(int j = 0; j < zoneMap->rangeX; j++)
        {
            (moveList + i * zoneMap->rangeX + j)->passed = false;
        }
    }

    (moveList + listA->node[0].y * zoneMap->rangeX + listA->node[0].x)->passed = true;
    (moveList + listA->node[0].y * zoneMap->rangeX + listA->node[0].x)->node = listA->node[0];
    
    (moveList + listB->node[0].y * zoneMap->rangeX + listB->node[0].x)->passed = true;
    (moveList + listB->node[0].y * zoneMap->rangeX + listB->node[0].x)->node = listB->node[0];
}

bool updatePathPlanner(MAP_GRID *zoneMap,NodeListType *listA,NodeListType *listB,LastNodeType *moveList,YAT_POINT pointA,YAT_POINT pointB,NodeListType *pathNode)
{
    bool finishFlag = false;

    YAT_POINT *currentNodeA = (YAT_POINT*)malloc(sizeof(YAT_POINT) * 2 * (zoneMap->rangeX + zoneMap->rangeY) );
    YAT_POINT *currentNodeB = (YAT_POINT*)malloc(sizeof(YAT_POINT) * 2 * (zoneMap->rangeX + zoneMap->rangeY) );

    YAT_POINT sameNodeA;
    YAT_POINT sameNodeB;

    int currentNumA = 0; 
    int currentNumB = 0; 

    for(int k = 0; k < listA->num; k++)
    {
        YAT_POINT lastNode = listA->node[k];

        for(int m = 0; m < 4; m++)
        {
            YAT_POINT moveVector;
            switch(m)
            {
                case 0:
                    moveVector.x = -1;
                    moveVector.y = 0;
                    break;
                case 1:
                    moveVector.x = 0;
                    moveVector.y = -1;
                    break;
                case 2:
                    moveVector.x = 1;
                    moveVector.y = 0;
                    break;
                case 3:
                    moveVector.x = 0;
                    moveVector.y = 1;
                    break;                           
                default:
                    ROS_ERROR("error move vector index.");
            }

            YAT_POINT nextNode;
            nextNode.x = lastNode.x + moveVector.x;
            nextNode.y = lastNode.y + moveVector.y;

            bool passFlag = (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->passed;
            bool detectFlag = (zoneMap->data[nextNode.y * zoneMap->rangeX + nextNode.x] == ID_CUT)?true:false;

            if(passFlag)
            {
                for(int n = 0; n < listB->num; n++)
                {
                    if( (listB->node[n].x == nextNode.x) && (listB->node[n].y == nextNode.y) )
                    {
                        sameNodeB = nextNode;
                        sameNodeA = lastNode;
                        finishFlag = true;
                        break;
                    }
                }
            }
            else
            {
                if(detectFlag)
                {
                    (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->passed = true;
                    (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->node = lastNode;
                    currentNodeA[currentNumA] = nextNode;
                    currentNumA++;
                }
            }
            if(finishFlag) break;
        }
        if(finishFlag) break;
    }

    listA->num = currentNumA;
    listA->step++;

    for(int m = 0; m < currentNumA; m++)
    {
        listA->node[m] = currentNodeA[m];
    }

    if(finishFlag)
    {
        YAT_POINT lastNode = sameNodeA;

        int numPoint = 0;
        pathNode->node[numPoint++] = lastNode;

        while( (lastNode.x != pointA.x) || (lastNode.y != pointA.y) )
        {
            lastNode = (moveList + lastNode.y * zoneMap->rangeX + lastNode.x)->node;
            pathNode->node[numPoint++] = lastNode;
        }

        for(int i = 0; 2 * i < numPoint - 1 ; i ++)
        {
            YAT_POINT midNode = pathNode->node[numPoint - 1 - i];
            pathNode->node[numPoint - 1 - i] = pathNode->node[i];
            pathNode->node[i] = midNode;
        }

        lastNode = sameNodeB;
        pathNode->node[numPoint++] = lastNode;

        while((lastNode.x != pointB.x) || (lastNode.y != pointB.y))
        {
            lastNode = (moveList + lastNode.y * zoneMap->rangeX + lastNode.x)->node;
            pathNode->node[numPoint++] = lastNode;
        }

        pathNode->num = numPoint;

        free(currentNodeA);
        free(currentNodeB);

        return true;
    }

    for(int k = 0; k < listB->num; k++)
    {
        YAT_POINT lastNode = listB->node[k];

        for(int m = 0; m < 4; m++)
        {
            YAT_POINT moveVector;
            switch(m)
            {
                case 0:
                    moveVector.x = -1;
                    moveVector.y = 0;
                    break;
                case 1:
                    moveVector.x = 0;
                    moveVector.y = -1;
                    break;
                case 2:
                    moveVector.x = 1;
                    moveVector.y = 0;
                    break;
                case 3:
                    moveVector.x = 0;
                    moveVector.y = 1;
                    break;                           
                default:
                    ROS_ERROR("error move vector index.");
            }

            YAT_POINT nextNode;
            nextNode.x = lastNode.x + moveVector.x;
            nextNode.y = lastNode.y + moveVector.y;

            bool passFlag = (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->passed;
            bool detectFlag = (zoneMap->data[nextNode.y * zoneMap->rangeX + nextNode.x] == ID_CUT)?true:false;
            if(passFlag)
            {
                for(int n = 0; n < listB->num; n++)
                {
                    if( (listA->node[n].x == nextNode.x) && (listA->node[n].y == nextNode.y) )
                    {
                        sameNodeA = nextNode;
                        sameNodeB = lastNode;
                        finishFlag = true;
                        break;
                    }
                }
            }
            else
            {
                if(detectFlag)
                {
                    (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->passed = true;
                    (moveList + nextNode.y * zoneMap->rangeX + nextNode.x)->node = lastNode;
                    currentNodeB[currentNumB] = nextNode;
                    currentNumB++;
                }
            }
            if(finishFlag) break;
        }
        if(finishFlag) break;
    }

    listB->num = currentNumB;
    listB->step++;

    for(int n = 0; n < currentNumB; n++)
    {
        listB->node[n] = currentNodeB[n];

        for(int m = 0; m < listA->num; m++)
        {
            if( (listB->node[n].x == listA->node[m].x) && (listB->node[n].y == listA->node[m].y) )
            {
                finishFlag = true;
            }
        }
    }

    if(finishFlag)
    {
        YAT_POINT lastNode = sameNodeA;

        int numPoint = 0;
        pathNode->node[numPoint++] = lastNode;

        while( (lastNode.x != pointA.x) || (lastNode.y != pointA.y) )
        {
            lastNode = (moveList + lastNode.y * zoneMap->rangeX + lastNode.x)->node;

            pathNode->node[numPoint++] = lastNode;
        }

        for(int i = 0; 2 * i < numPoint - 1 ; i ++)
        {
            YAT_POINT midNode = pathNode->node[numPoint - 1 - i];
            pathNode->node[numPoint - 1 - i] = pathNode->node[i];
            pathNode->node[i] = midNode;
        }

        lastNode = sameNodeB;
        pathNode->node[numPoint++] = lastNode;

        while((lastNode.x != pointB.x) || (lastNode.y != pointB.y))
        {
            lastNode = (moveList + lastNode.y * zoneMap->rangeX + lastNode.x)->node;
            pathNode->node[numPoint++] = lastNode;
        }

        pathNode->num = numPoint;
    }

    free(currentNodeA);
    free(currentNodeB);

    if( (!finishFlag) && ( (currentNumA == 0)||(currentNumB == 0) ) )
    {
        pathNode->num = 0;
        finishFlag = true;
    }

    return finishFlag;
}

bool pathPlanner(MAP_GRID *zoneMap,YAT_POINT pointA,YAT_POINT pointB,YAT_POINT &pointC)
{
    NodeListType *nodeListA = (NodeListType*)malloc(sizeof(NodeListType));
    NodeListType *nodeListB = (NodeListType*)malloc(sizeof(NodeListType));

    LastNodeType *fatherList= (LastNodeType*)malloc(sizeof(LastNodeType) * zoneMap->rangeX * zoneMap->rangeY);

    NodeListType *pathNode = (NodeListType*)malloc(sizeof(NodeListType));

    initialNodeLst(nodeListA,nodeListB,fatherList,zoneMap,pointA,pointB,pathNode);

    bool keepMission = true;

    ros::Time startTime = ros::Time::now();

    while(keepMission)
    {
        if(updatePathPlanner(zoneMap,nodeListA,nodeListB,fatherList,pointA,pointB,pathNode))
        {
            double dt = ros::Time::now().toSec() - startTime.toSec();

            ROS_INFO("the path planner cost time is %f second.",dt);
            
            for(int k = 0 ; k < pathNode->num ; k++)
            {
                // ROS_INFO("the NO.%d path node point is x = %d, y = %d",k,pathNode->node[k].x,pathNode->node[k].y); 
            }

            keepMission = false;
        }
    }

    bool getPathPoint = false;

    if(pathNode->num > 0)
    {
        YAT_POINT targetPoint = pathNode->node[0];

        for(int k = 1 ; k < pathNode->num ; k++)
        {
            if(enableStraightArrive(zoneMap,pointA,pathNode->node[k]))
            {
               targetPoint = pathNode->node[k];
            }
            else
            {
                // ROS_INFO("the target No.%d point.",k);
                break;
            }
        }

        pointC = targetPoint;

        // ROS_INFO("the target point is : x = %d, y = %d .",targetPoint.x,targetPoint.y);

        getPathPoint = true;
    }

    free(nodeListA);
    free(nodeListB);
    free(fatherList);
    free(pathNode);

    return getPathPoint;
}

bool getPathPlanner(MAP_GRID *zoneMap,CutZoneType *zoneMsg,YAT_POINTF targetPoint)
{
    gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINT maxPoint;
    YAT_POINT minPoint;

    findMaximumVaule(zoneMap,maxPoint,minPoint);

    YAT_POINTF offsetPoint = rasterToWorld(minPoint,parametre);

    MAP_GRID *mapGrid = (MAP_GRID *)malloc(sizeof(MAP_GRID));

    mapGrid->scale = zoneMap->scale;
    mapGrid->rangeX = maxPoint.x - minPoint.x + 1;
    mapGrid->rangeY = maxPoint.y - minPoint.y + 1;
    mapGrid->offsetX = offsetPoint.x;
    mapGrid->offsetY = offsetPoint.y;

    mapGrid->data = (int *)malloc(sizeof(int)* (mapGrid->rangeX * mapGrid->rangeY));

    for(int j = 0; j < mapGrid->rangeY ; j++)
    {
        for(int i = 0; i < mapGrid->rangeX ; i++)
        {
            mapGrid->data[j * mapGrid->rangeX + i] = zoneMap->data[(j + minPoint.y)* zoneMap->rangeX + (i + minPoint.x)];
        }
    }
    
    dilateMap(mapGrid, 0.5);

    spdlog::get("robot_state")->info("get next zone path :  target Point -> x = {} , y = {}.",targetPoint.x,targetPoint.y);

    gridMapType mapParam = getGridParametre(mapGrid);

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    YAT_POINTF startPoint = worldPointToMap(zoneMsg,robotPoint);
    YAT_POINT startXY = worldToRaster(startPoint,mapParam);
    YAT_POINT pathPointA = findNestEnablePoint(mapGrid,startXY);

    YAT_POINTF endingPoint = worldPointToMap(zoneMsg,targetPoint);
    YAT_POINT endingXY = worldToRaster(endingPoint,mapParam);
    YAT_POINT pathPointB = findNestEnablePoint(mapGrid,endingXY);

    YAT_POINT targetXY = pathPointB;

    bool getPathSymbol = false;

    if(pathPlanner(mapGrid,pathPointA,pathPointB,targetXY))
    {
        YAT_POINTF pathPoint = rasterToWorld(targetXY,mapParam);
        sensorData.pathPoint[0] = mapPointToWorld(zoneMsg,pathPoint);
        spdlog::get("robot_state")->info("get mid path point to next point: x = {}, y = {}.",sensorData.pathPoint[0].x,sensorData.pathPoint[0].y);
        getPathSymbol = true;
    }

    free(mapGrid->data);
    free(mapGrid);

    return getPathSymbol;
}

bool lengthWiseMotion(YAT_POINTF nextPoint,YAT_POINTF lastPoint,YAT_POINTF &commandVelocity,bool &finishFlag)
{
    static int motionStep = 0;
    bool motionFinish = false;

    static YAT_POINTF pointB = nextPoint;
    static YAT_POINTF pointA = lastPoint;

    static YAT_POINTF startPoint = lastPoint;
    static double moveDistance = 50;
    static int moveTimes = 0;

    YAT_POINTF robotPoint;
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0)
    {
        pointB = nextPoint;
        pointA = lastPoint;

        moveDistance = 50;

        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        commandVelocity = turning(pointB,pointA);

        if(turnedTarget(pointB,pointA,5.0))
        {
            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            pointB = nextPoint;
            pointA = lastPoint;

            if(detectData.rightSide != detectData.leftSide)
            {
                motionStep = 3;
            }
            else
            {
                motionStep = 5;
            }
        }
    }

    // move to inside
    if( motionStep == 3 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.05))
        {
            pointB = nextPoint;
            pointA = lastPoint;

            moveDistance = 0;
            motionStep = 15;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_ERROR("move to inside : robot meet bumped.");

            pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
            pointB.y = robotPose.y - 0.2 * cos(robotPose.theta);

            pointB.x = robotPose.x;
            pointB.y = robotPose.y;

            motionStep = 4;
        }

        if(detectData.rightSide && detectData.leftSide)
        {
            motionStep = 5;
        }
        else if((!detectData.rightSide) && (!detectData.leftSide))
        {
            moveDistance = 0;
            motionStep = 15;
        }
    }

    // move back
    if( motionStep == 4 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.1))
        {
            moveDistance = 0;
            motionStep = 15;
        }
    }

    // move forward block
    if( motionStep == 5 )
    {
        startPoint.x = robotPose.x;
        startPoint.y = robotPose.y;

        motionStep = 6;
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            motionStep = 7;
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");
            motionStep = 7;
        }

        if( arrivedTarget(pointB, pointA, 0.05) )
        {
            ROS_INFO("arrive length wise target point.");
            moveDistance = deltaDistance(robotPoint , pointA);
            motionStep = 15;
        }
    }

    if( motionStep == 7 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            float pathDistance = deltaDistance(nextPoint , startPoint);

            YAT_POINTF pathVector;
            pathVector.x = (nextPoint.x - lastPoint.x) / pathDistance;
            pathVector.y = (nextPoint.y - lastPoint.y) / pathDistance;

            YAT_POINTF targetVector;
            targetVector.x = robotPose.x - startPoint.x;
            targetVector.y = robotPose.y - startPoint.y;

            moveDistance = targetVector.x * pathVector.x + targetVector.y * pathVector.y;

            pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
            pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);
            
            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 8;
        }
    }

    if( motionStep == 8 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.1))
        {
            pointB = nextPoint;
            pointA = lastPoint;

            motionStep = 15;
        }
    }

    if( motionStep == 15 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            finishFlag = (fabs(moveDistance) < 0.2)?true:false;
            if(finishFlag) ROS_ERROR("move forward length wise target, robot move distance is less than 0.2.");

            moveTimes ++;
            if(moveTimes == 1)
            {
                finishFlag = false;
            }

            if(finishFlag)
            {
                moveTimes = 0;
            }

            motionStep = 0;
            motionFinish = true;
        }
    }

    // ROS_INFO("length Wise motion -> motion step is : %d.",motionStep);
    spdlog::get("robot_state")->info("lengthwise detect step is : {}, move times is {}.",motionStep,moveTimes);

    return motionFinish;
}

bool transversalMotion(YAT_POINTF nextPoint,YAT_POINTF lastPoint,YAT_POINTF &commandVelocity,CutZoneType *zoneMsg,bool finishFlag)
{
    static int motionStep = 0;
    bool motionFinish = false;

    static YAT_POINTF pointB = nextPoint;
    static YAT_POINTF pointA = lastPoint;

    static bool clockwise = false;
    static bool turnWise = false;
    static YAT_POINTF startPoint = lastPoint;

    static float totalAngle = 0;
    static float lastAngle = 0;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0)
    {
        totalAngle = 0;

        pointB = nextPoint;
        pointA = lastPoint;

        float targetAngle = atan2(nextPoint.y - lastPoint.y,nextPoint.x - lastPoint.x);
        turnWise = (constrainAngle(targetAngle - robotPose.theta) > 0)?true:false;

        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        commandVelocity = turning(pointB,pointA);

        if(turnedTarget(pointB,pointA,5.0))
        {
            lastAngle = robotPose.theta;

            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            if(detectData.rightSide && detectData.leftSide)
            {
                motionStep = 3;
            }
            else
            {
                ROS_INFO("the detector is outside, then find the boundary for moving.");
                motionStep = 5;
            }

            startPoint.x = robotPose.x;
            startPoint.y = robotPose.y;
        }
    }

    // move forward block
    if( motionStep == 3 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if( motionStep == 4 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            motionStep = 15;
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");
            motionStep = 5;
        }

        if( arrivedTarget(pointB, pointA, 0) )
        {
            motionStep = 15;
        }
    }

    if( motionStep == 5 )
    {
        float targetAngleA = atan2(nextPoint.y - lastPoint.y,nextPoint.x - lastPoint.x);
        float targetAngleB = constrainAngle(zoneMsg->alongAngle + PI);

        float turningAngle = turnWise ? constrainAngle(robotPose.theta + PI/3) : constrainAngle(robotPose.theta - PI/3); 

        turning(turningAngle,commandVelocity);

        float deltaThetaA = constrainAngle(targetAngleA - robotPose.theta );
        float deltaThetaB = constrainAngle(targetAngleB - robotPose.theta );

        bool flagA = (fabs(deltaThetaA) < 90 * DegreeToRad)?true:false;
        bool flagB = (fabs(deltaThetaB) < 90 * DegreeToRad)?true:false;

        if( flagA && flagB )
        {
            if(detectData.rightSide != detectData.leftSide)
            {
                sensorData.updateAlong = true;
                clockwise = detectData.rightSide;
                motionStep = 6;
            }

            if(detectData.rightSide && detectData.leftSide)
            {
                motionStep = 4;
            }

            ROS_INFO("find the boundary direction, then along the boundary");
        }

        totalAngle = totalAngle + fabs(constrainAngle(robotPose.theta - lastAngle));
        lastAngle = robotPose.theta;

        if(fabs(totalAngle) > 0.5 * PI)
        {
            motionStep = 10;
            ROS_ERROR("did not find the boundary direction, then turn to the random mode");
        }
    }

    if(motionStep == 6)
    {
        bool turningFlag = false;
        if(moveAlongBoundary(clockwise,commandVelocity,turningFlag))
        {
            motionStep = 10;
            ROS_ERROR("move along boundary failure, then turn to the random mode");
        }

        if( arrivedTarget(nextPoint, lastPoint, 0.05) )
        {
            ROS_INFO("arrive at the transversal point.");
            motionStep = 15;
        }

        if(sqrt(pow(startPoint.x - robotPose.x,2) +  pow(startPoint.y - robotPose.y,2)) > 0.3)
        {
            motionStep = 15;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            motionStep = 15;
        }
    }

    if( motionStep == 10 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            finishFlag = true;

            motionStep = 0;
            motionFinish = true;
        }
    }

    if( motionStep == 15 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            finishFlag = false;

            motionStep = 0;
            motionFinish = true;
        }
    }

    // ROS_INFO("transversal motion -> motion step is : %d.",motionStep);
    spdlog::get("robot_state")->info("transversal detect step is : {}.",motionStep);

    return motionFinish;
}

bool findEnableYaw(CutZoneType *zoneMsg,YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;

    YAT_POINTF robotPoint;
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF pointB = robotPoint;
    static YAT_POINTF pointA = robotPoint;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    static double targetAngleA = constrainAngle(zoneMsg->yaw + PI/2);
    static double targetAngleB = constrainAngle(zoneMsg->yaw - PI/2);

    static bool clockwise = false;
    static double lastAngle = 0;
    static double totalAngle = 0;

    static double enableYawA = robotPose.theta;
    static double enableYawB = robotPose.theta;
    static bool findEnable = false;

    if( motionStep == 0)
    {
        totalAngle = 0;
        lastAngle = robotPose.theta;

        findEnable = false;

        motionStep = 1;

        if(detectData.rightSide && detectData.leftSide)
        {
            if(fabs(constrainAngle(targetAngleA - robotPose.theta)) < 5 * DegreeToRad)
            {
                motionStep = 10;
            }
            
            if(fabs(constrainAngle(targetAngleB - robotPose.theta)) < 5 * DegreeToRad)
            {
                motionStep = 10;
            }
        }
    }

    if( motionStep == 1 )
    {
        targetAngleA = constrainAngle(zoneMsg->yaw + PI/2);
        targetAngleB = constrainAngle(zoneMsg->yaw - PI/2);

        if((detectData.rightSide) && (!detectData.leftSide))
        {
            clockwise = true;
        }
        else if((!detectData.rightSide) && (detectData.leftSide))
        {
            clockwise = false;
        }
        else
        {
            // the detector is outside
            if( fabs(constrainAngle(targetAngleA - robotPose.theta)) < fabs(constrainAngle(targetAngleB - robotPose.theta)) )
            {
                clockwise = (constrainAngle(targetAngleA - robotPose.theta) < 0)?true:false;
            }
            else
            {
                clockwise = (constrainAngle(targetAngleB - robotPose.theta) < 0)?true:false;
            }
        }

        motionStep = 2;
    }

    if( motionStep == 2 )
    {
        double targetAngle = clockwise ?( constrainAngle(robotPose.theta - PI/3) ):(constrainAngle(robotPose.theta + PI/3)); 

        turning(targetAngle,commandVelocity);

        if(detectData.rightSide && detectData.leftSide)
        {
            if(fabs(constrainAngle(targetAngleA - robotPose.theta)) < 5 * DegreeToRad)
            {
                ROS_INFO("find the Enablde Yaw : flag 1");
                motionStep = 10;
            }
            
            if(fabs(constrainAngle(targetAngleB - robotPose.theta)) < 5 * DegreeToRad)
            {
                ROS_INFO("find the Enablde Yaw : flag 2");

                motionStep = 10;
            }

            findEnable = true;
        }

        if(detectData.rightSide != detectData.leftSide)
        {
            if(detectData.rightSide) enableYawA = robotPose.theta;
            if(detectData.leftSide) enableYawB = robotPose.theta;
        }

        totalAngle = totalAngle + fabs(constrainAngle(robotPose.theta - lastAngle));
        lastAngle = robotPose.theta;

        if(fabs(totalAngle) > 2 * PI)
        {
            if(findEnable)
            {
                totalAngle = 0;
                motionStep = 3;  // did not find the right direction, then turn to the random angle
            }
            else
            {
                zoneMsg->type = 0;
                totalAngle = 0;
                motionStep = 10;  // did not find the right direction, then turn to the random angle
            }
        }
    }

    if( motionStep == 3 )
    {
        targetAngleA = constrainAngle(enableYawA + 0.5 * constrainAngle(enableYawB - enableYawA));
        targetAngleB = constrainAngle(targetAngleA + PI);

        if((detectData.rightSide) && (!detectData.leftSide))
        {
            clockwise = true;
        }
        else if((!detectData.rightSide) && (detectData.leftSide))
        {
            clockwise = false;
        }
        else
        {
            // the detector is outside
            if( fabs(constrainAngle(targetAngleA - robotPose.theta)) < fabs(constrainAngle(targetAngleB - robotPose.theta)) )
            {
                clockwise = (constrainAngle(targetAngleA - robotPose.theta) < 0)?true:false;
            }
            else
            {
                clockwise = (constrainAngle(targetAngleB - robotPose.theta) < 0)?true:false;
            }
        }

        motionStep = 4;
    }

    if( motionStep == 4 )
    {
        double targetAngle = clockwise ?( constrainAngle(robotPose.theta - PI/3) ):(constrainAngle(robotPose.theta + PI/3)); 

        turning(targetAngle,commandVelocity);

        if(detectData.rightSide && detectData.leftSide)
        {
            if(fabs(constrainAngle(targetAngleA - robotPose.theta)) < 5 * DegreeToRad)
            {
                ROS_INFO("find the Enablde Yaw : flag 1");
                motionStep = 5;
            }
            
            if(fabs(constrainAngle(targetAngleB - robotPose.theta)) < 5 * DegreeToRad)
            {
                ROS_INFO("find the Enablde Yaw : flag 2");

                motionStep = 5;
            }
        }

        if(detectData.rightSide != detectData.leftSide)
        {
            if(detectData.rightSide) enableYawA = robotPose.theta;
            if(detectData.leftSide) enableYawB = robotPose.theta;
        }

        totalAngle = totalAngle + fabs(constrainAngle(robotPose.theta - lastAngle));
        lastAngle = robotPose.theta;

        if(fabs(totalAngle) > 2 * PI)
        {
            zoneMsg->type = 0;
            totalAngle = 0;
            motionStep = 10;  // did not find the right direction, then turn to the random angle
        }
    }

    if( motionStep == 5 )
    {
        pointB.x = robotPose.x + 0.5 * cos(robotPose.theta);
        pointB.y = robotPose.y + 0.5 * sin(robotPose.theta);

        pointA.x = robotPose.x;
        pointA.y = robotPose.y;

        motionStep = 6;
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            motionStep = 0;
        }

        if((!detectData.rightSide) && (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");
            motionStep = 0;
        }

        if( arrivedTarget(pointB, pointA, 0.05) )
        {
            motionStep = 0;
        }
    }

    if( motionStep == 10 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 0;
            motionFinish = true;
        }
    }

    spdlog::get("robot_state")->info("find enable yaw: step is {}.",motionStep);

    return motionFinish;
}

bool gotoNextZone(CutZoneType *zoneMsg,MAP_GRID *zoneMap,YAT_POINTF &commandVelocity)
{
    static int motionStep = 0;
    bool motionFinish = false;

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF pointB = robotPoint;
    static YAT_POINTF pointA = robotPoint; 
    static YAT_POINTF targetPoint = robotPoint;

    static bool clockwise = false;

    enum MOVE_TYPE
    {
        TYPE_STRAIGHT = 0, TYPE_PATH = 1, TYPE_RANDOM = 2,
    };

    static int moveFlag = TYPE_STRAIGHT;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0 ) 
    {
        sensorData.randomStart = ros::Time::now();

        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        bool findTargetPoint = false;

        if(zoneMsg->num == 1)
        {
            float delta_AB = distanceCenterLine(zoneMsg,robotPoint);

            pointB.x = robotPose.x + (3 - delta_AB) * cos(zoneMsg->yaw + PI/2);
            pointB.y = robotPose.y + (3 - delta_AB) * sin(zoneMsg->yaw + PI/2);

            sensorData.pathPoint[0] = pointB;

            motionStep = 2; // planner motion
        }
        else 
        {
            if(zoneMsg->num >= 1)
            {
                findTargetPoint = findTopPoint(zoneMap,zoneMsg,targetPoint);
            }
            else
            {
                findTargetPoint = findBottomPoint(zoneMap,zoneMsg,targetPoint);
            }

            if(findTargetPoint)
            {
                if(getPathPlanner(zoneMap,zoneMsg,targetPoint))
                {
                    pointB = sensorData.pathPoint[0];
                    motionStep = 2;
                }
                else if(getRandomPath(zoneMap,zoneMsg,targetPoint))
                {
                    pointB = sensorData.pathPoint[0];
                    motionStep = 2;
                }
                else
                {
                    motionStep = 8;
                }

                pointA = robotPoint;
            }
            else
            {
                zoneMsg->arrive = false;
                motionStep = 12;
            }
        }

        pointA = robotPoint;
    }

    if( motionStep == 2 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 3;
        }
    }

    if( motionStep == 3 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if( motionStep == 4 )
    {
        commandVelocity = move_forward(pointB,pointA);

        float delta_AB = sqrt(pow(robotPose.x - pointA.x,2) + pow(robotPose.y - pointA.y,2));
        bool enableBack = (delta_AB > 0.2)?true:false;

        if(arrivedTarget(pointB, pointA, 0.2))
        {
            ROS_INFO("arrive at the remain zone boundary.");

            motionStep = 1;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 8;
            }
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");
            motionStep = 6;
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 8;
            }
        }
    }

    if( motionStep == 5 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 6;
        }
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.20))
        {
            motionStep = 7;
        }
    }

    if(motionStep == 7)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 1;
        }
    }

    if( motionStep == 8 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            float targetAngle  = constrainAngle(fabs(rand() % 360) * DegreeToRad);

            pointB.x = robotPose.x + 5.0 * cos(targetAngle);
            pointB.y = robotPose.y + 5.0 * sin(targetAngle);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 9;
        }
    }

    if( motionStep == 9 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 10;
        }
    }

    if( motionStep == 10 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if(motionStep == 11)
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 12;
        }
    }

/************************************************************/



    if(fabs(ros::Time::now().toSec() - sensorData.randomStart.toSec()) > 180)
    {
        motionStep = 12;
        zoneMsg->arrive = false;
    }

    double distanceToZone = distanceCenterLine(zoneMsg,robotPoint);
    if(fabs(distanceToZone) < 3.0)
    {
        zoneMsg->arrive = true;
        motionStep = 12;
    }

/************************************************************/

    if( motionStep == 12 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        motionStep = 0;
        motionFinish = true;
    }

    spdlog::get("robot_state")->info("go to next point : x = {}, y = {}.",pointB.x,pointB.y);
    spdlog::get("robot_state")->info("go to next zone : the step is {}.",motionStep);

    return motionFinish;
}

bool gotoNextSideZone(CutZoneType *zoneMsg,MAP_GRID *zoneMap,YAT_POINTF &commandVelocity,bool &finishFlag)
{
    static int motionStep = 0;
    bool motionFinish = false;

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF pointB = robotPoint;
    static YAT_POINTF pointA = robotPoint; 

    static YAT_POINTF remainPoint = robotPoint;

    static bool clockwise = false;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0 ) 
    {
        sensorData.randomStart = ros::Time::now();

        motionStep = 1;
    }

    if( motionStep == 1 )
    {
        if(findRemainZone(zoneMap,zoneMsg,remainPoint))
        {

            if(getPathPlanner(zoneMap,zoneMsg,remainPoint))
            {
                pointB = sensorData.pathPoint[0];
                motionStep = 2;
            }
            else if(getRandomPath(zoneMap,zoneMsg,remainPoint))
            {
                pointB = sensorData.pathPoint[0];
                motionStep = 2;
            }
            else
            {
                motionStep = 8;
            }

            pointA = robotPoint;
        }
        else
        {
            motionStep = 12;
            finishFlag = false;
        }
    }

    if( motionStep == 2 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 3;
        }
    }

    if( motionStep == 3 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if( motionStep == 4 )
    {
        commandVelocity = move_forward(pointB,pointA);

        float delta_AB = sqrt(pow(robotPose.x - pointA.x,2) + pow(robotPose.y - pointA.y,2));
        bool enableBack = (delta_AB > 0.2)?true:false;

        if(arrivedTarget(pointB, pointA, 0.05))
        {
            ROS_INFO("arrive at the remain zone boundary.");

            motionStep = 1;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 8;
            }
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");
            motionStep = 6;
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 8;
            }
        }
    }

    if( motionStep == 5 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 6;
        }
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.20))
        {
            motionStep = 7;
        }
    }

    if(motionStep == 7)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 1;
        }
    }

    if( motionStep == 8 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            float targetAngle  = constrainAngle(fabs(rand() % 360) * DegreeToRad);

            pointB.x = robotPose.x + 5.0 * cos(targetAngle);
            pointB.y = robotPose.y + 5.0 * sin(targetAngle);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 9;
        }
    }

    if( motionStep == 9 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 10;
        }
    }

    if( motionStep == 10 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }


/************************************************************/

    if(fabs(ros::Time::now().toSec() - sensorData.randomStart.toSec()) > 90)
    {
        motionStep = 12;
        finishFlag = false;
    }

    double remainToZone = distanceCenterLine(zoneMsg,remainPoint);
    double currentToZone = distanceCenterLine(zoneMsg,robotPoint);

    YAT_POINTF targetVector;
    targetVector.x = remainPoint.x - robotPoint.x;
    targetVector.y = remainPoint.y - robotPoint.y;

    if(zoneMsg->side == 1)
    {
        float judgeData = targetVector.x * cos(zoneMsg->yaw) + targetVector.y * sin(zoneMsg->yaw);
        if((judgeData < 0.5)&&fabs(currentToZone)< 3)
        {
            motionStep = 12;
            finishFlag = true;
        }
    }
    else if(zoneMsg->side == 2)
    {
        float judgeData = targetVector.x * cos(zoneMsg->yaw + PI) + targetVector.y * sin(zoneMsg->yaw + PI);
        if((judgeData < 0.5)&&fabs(currentToZone)< 3)
        {
            motionStep = 12;
            finishFlag = true;
        }
    }
    else
    {
        motionStep = 12;
        finishFlag = false;
    }

/************************************************************/

    if( motionStep == 12 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        motionStep = 0;
        motionFinish = true;
    }

    spdlog::get("robot_state")->info("go to next side point : x = {}, y = {}.",pointB.x,pointB.y);
    spdlog::get("robot_state")->info("go to next side zone : the step is {}.",motionStep);

    return motionFinish;
}

bool gotoRemainZone(CutZoneType *zoneMsg,MAP_GRID *zoneMap,YAT_POINTF &commandVelocity,bool &finishFlag)
{
    static int motionStep = 0;
    bool motionFinish = false;

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF pointB = robotPoint;
    static YAT_POINTF pointA = robotPoint; 

    static YAT_POINTF remainPoint = robotPoint;

    static bool clockwise = false;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0 ) 
    {
        sensorData.randomStart = ros::Time::now();

        if(findRemainZone(zoneMap,zoneMsg,remainPoint))
        {
            pointB = remainPoint;
            pointA = robotPoint;

            motionStep = 2; // planner motion
        }
        else
        {
            motionStep = 12;
            finishFlag = false;
        }

    }

    if( motionStep == 1 )
    {
        if(getPathPlanner(zoneMap,zoneMsg,remainPoint))
        {
            pointB = sensorData.pathPoint[0];
        }
        else if(getRandomPath(zoneMap,zoneMsg,remainPoint))
        {
            pointB = sensorData.pathPoint[0];
        }
        else
        {
            pointB = remainPoint;
        }

        pointA = robotPoint;

        motionStep = 2;
    }

    if( motionStep == 2 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 3;
        }
    }

    if( motionStep == 3 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if( motionStep == 4 )
    {
        commandVelocity = move_forward(pointB,pointA);

        float delta_AB = sqrt(pow(robotPose.x - pointA.x,2) + pow(robotPose.y - pointA.y,2));
        bool enableBack = (delta_AB > 0.2)?true:false;

        if(arrivedTarget(pointB, pointA, 0.05))
        {
            ROS_INFO("arrive at the remain zone boundary.");

            motionStep = 1;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 8;
            }
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");

            pointB.x = robotPose.x + 0.1 * cos(robotPose.theta);
            pointB.y = robotPose.y + 0.1 * sin(robotPose.theta);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;
            
            motionStep = 11;
        }
    }

    if( motionStep == 5 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 6;
        }
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.20))
        {
            motionStep = 7;
        }
    }

    if(motionStep == 7)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 1;
        }
    }

    if( motionStep == 8 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            float targetAngle  = constrainAngle(fabs(rand() % 360) * DegreeToRad);

            pointB.x = robotPose.x + 5.0 * cos(targetAngle);
            pointB.y = robotPose.y + 5.0 * sin(targetAngle);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 9;
        }
    }

    if( motionStep == 9 )
    {
        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);

        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 10;
        }
    }

    if( motionStep == 10 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 4;
        }
    }

    if( motionStep == 11 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.05))
        {
            motionStep = 12;
        }
    }

    if( motionStep == 12 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            YAT_POINTF targetVector;
            targetVector.x = remainPoint.x - robotPoint.x;
            targetVector.y = remainPoint.y - robotPoint.y;

            float judgeData = targetVector.x * cos(zoneMsg->yaw + PI/2) + targetVector.y * sin(zoneMsg->yaw + PI/2);

            
            int alongSide = getAlongBoundaryWise(zoneMap,zoneMsg,robotPoint);
            
            if(zoneMsg->side == 1)
            {
                alongSide = 1;
            }

            motionStep = 13;

            if(alongSide == 1)
            {
                if(judgeData > 0)
                {
                    clockwise = false;
                }
                else
                {
                    clockwise = true;
                }
            }
            else if(alongSide == -1)
            {
                if(judgeData < 0)
                {
                    clockwise = false;
                }
                else
                {
                    clockwise = true;
                }
            }
            else
            {
                motionStep = 1;
            }
        }
    }

    if( motionStep == 13 )
    {
        bool turningFlag = false;
        if(moveAlongBoundary(clockwise,commandVelocity,turningFlag))
        {
            motionStep = 7;
            ROS_ERROR("move along boundary failure, then turn to the random mode");
        }

        YAT_POINTF targetVector;
        targetVector.x = remainPoint.x - robotPoint.x;
        targetVector.y = remainPoint.y - robotPoint.y;

        float judgeData = targetVector.x * cos(zoneMsg->yaw + PI/2) + targetVector.y * sin(zoneMsg->yaw + PI/2);
        float targetAngle = (judgeData > 0)?constrainAngle(zoneMsg->yaw + PI/2):constrainAngle(zoneMsg->yaw - PI/2);

        if(turningFlag)
        {
            pointB.x = robotPose.x + judgeData * cos(zoneMsg->yaw + PI/2);
            pointB.y = robotPose.y + judgeData * sin(zoneMsg->yaw + PI/2);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;

            motionStep = 2;
        }
    }

/************************************************************/

    if(fabs(ros::Time::now().toSec() - sensorData.randomStart.toSec()) > 180)
    {
        motionStep = 15;
        finishFlag = false;
    }

    YAT_POINTF targetVector;
    targetVector.x = robotPoint.x - remainPoint.x;
    targetVector.y = robotPoint.y - remainPoint.y;

    float deltaHigh = targetVector.x * cos(zoneMsg->yaw + PI/2) + targetVector.y * sin(zoneMsg->yaw + PI/2);
    float targetAngle = (deltaHigh > 0)?constrainAngle(zoneMsg->yaw + PI/2):constrainAngle(zoneMsg->yaw - PI/2);

    if(zoneMsg->side == 1)
    {
        float judgeData = targetVector.x * cos(zoneMsg->yaw) + targetVector.y * sin(zoneMsg->yaw);
        if((judgeData > - 0.3)&&(fabs(deltaHigh) < 0.2))
        {
            motionStep = 15;
            finishFlag = true;
        }
    }
    else if(zoneMsg->side == 2)
    {
        float judgeData = targetVector.x * cos(zoneMsg->yaw + PI) + targetVector.y * sin(zoneMsg->yaw + PI);
        if((judgeData < 0.3)&&(fabs(deltaHigh) < 0.2))
        {
            motionStep = 15;
            finishFlag = true;
        }
    }
    else
    {
        motionStep = 15;
        finishFlag = false;
    }

/************************************************************/

    if( motionStep == 15 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        motionStep = 0;
        motionFinish = true;
    }

    spdlog::get("robot_state")->info("go to remain point : x = {}, y = {}.",pointB.x,pointB.y);
    spdlog::get("robot_state")->info("go to remain zone : the step is {}.",motionStep);

    return motionFinish;
}

bool gobackHome()
{
    static int motionStep = 0;
    bool motionFinish = false;

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF pointB = robotPoint;
    static YAT_POINTF pointA = robotPoint; 

    YAT_POINTF commandVelocity;

    static bool clockwise = false;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if( motionStep == 0 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        pointB.x = robotPoint.x + 1000;
        pointB.y = robotPoint.y;

        pointA = robotPoint; 

        motionStep = 1;
    }

    if( motionStep == 1 )
    {

        float targetAngle = atan2(pointB.y - pointA.y,pointB.x - pointA.x);
        
        if(turning(targetAngle,commandVelocity))
        {
            motionStep = 2;
        }
    }

    if( motionStep == 2 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 3;
        }
    }

    if( motionStep == 3 )
    {
        commandVelocity = move_forward(pointB,pointA);

        bool enableBack = (deltaDistance(robotPoint,pointA) > 0.2)?true:false;

        if(arrivedTarget(pointB, pointA, 0.20))
        {
            motionStep = 7;
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump.");
            if(enableBack)
            {
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
            else
            {
                motionStep = 7;
            }
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            ROS_INFO("meet the edge.");

            pointB.x = robotPose.x + 0.1 * cos(robotPose.theta);
            pointB.y = robotPose.y + 0.1 * sin(robotPose.theta);

            pointA.x = robotPose.x;
            pointA.y = robotPose.y;
            
            motionStep = 8;
        }

    }

    if( motionStep == 5 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 6;
        }
    }

    if( motionStep == 6 )
    {
        commandVelocity = move_back(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.20))
        {
            motionStep = 7;
        }
    }

    if(motionStep == 7)
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            float targetAngle = constrainAngle(DegreeToRad * (rand() % 360) );

            pointB.x = robotPoint.x + 1000 * cos(targetAngle);
            pointB.y = robotPoint.y + 1000 * sin(targetAngle);

            pointA = robotPoint; 

            motionStep = 1;
        }
    }

    if( motionStep == 8 )
    {
        commandVelocity = move_forward(pointB,pointA);

        if(arrivedTarget(pointB, pointA, 0.5))
        {
            motionStep = 9;
        }
    }

    if( motionStep == 9 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 10;
        }
    }

    if( motionStep == 10 )
    {
        bool turningFlag = false;
        if(moveAlongBoundary(false,commandVelocity,turningFlag))
        {
            motionStep = 7;
            ROS_ERROR("move along boundary failure, then turn to the random mode");
        }

        if(detectData.leftBump || detectData.rightBump)
        {
            ROS_INFO("meet the bump, finish the mission.");
            motionStep = 11; // motionStep = 6;

            sensorData.updateAlong = true;

        }
    }

    if( motionStep == 11 )
    {

        pointB.x = robotPose.x + 0.06 * cos(robotPose.theta);
        pointB.y = robotPose.y + 0.06 * sin(robotPose.theta);

        pointA.x = robotPose.x;
        pointA.y = robotPose.y;

        commandVelocity = move_forward(pointB,pointA);

        bool isCharging = false;

        ROS_ERROR("charge detecting");

        if(chargeDetect(isCharging))
        {
            if(isCharging)
            {
                ROS_ERROR("the robot is charging , the mission is finished.");
                motionStep = 9;
            }
            else
            {
                ROS_ERROR("the robot is not charging , go to the random position.");
                pointB.x = robotPose.x - 0.2 * cos(robotPose.theta);
                pointB.y = robotPose.y - 0.2 * sin(robotPose.theta);

                pointA.x = robotPose.x;
                pointA.y = robotPose.y;

                motionStep = 5;
            }
        }
    }

    if( motionStep == 9 )
    {
        commandVelocity.x = 0;
        commandVelocity.y = 0;

        if(robotStop)
        {
            motionStep = 0;
            motionFinish = true;
        }
    }

    /************************************************/
    msg_vel.linear.x = commandVelocity.x;     // velocity control
    msg_vel.angular.z = commandVelocity.y;    // angle control

    spdlog::get("robot_state")->info("motion detect -> next point: x = {}, y = {} .",pointB.x,pointB.y);
    spdlog::get("robot_state")->info("motion detect -> last point: x = {}, y = {} .",pointA.x,pointA.y);
    spdlog::get("robot_state")->info("motion detect -> control msg: v = {}, w = {} .",commandVelocity.x,commandVelocity.y);
    spdlog::get("robot_state")->info("motion detect -> detector: left = {}, right = {} .",detectData.leftSide,detectData.rightSide);
    spdlog::get("robot_state")->info("motion detect -> detector:  left = {}, right = {} .",detectData.leftBump,detectData.rightBump);
    spdlog::get("robot_state")->info("chargeing current {}, motion detect step is : {}.\n",detectData.chargeCurrent,motionStep);

    vel_pub.publish(msg_vel);

    return motionFinish;
}

bool motionDetect(MAP_GRID *zoneMap, CutZoneType *zoneMsg)
{
    static int motionType = 0;
    static int motionStep = 0;
    static int motionSide = 0;

    YAT_POINTF robotPoint; 
    robotPoint.x = robotPose.x;
    robotPoint.y = robotPose.y;

    static YAT_POINTF lastPoint = robotPoint;
    static YAT_POINTF nextPoint = robotPoint;

    static YAT_POINTF breakPoint = robotPoint;

    bool taskFinish = false;
    bool detectFinish = false;

    YAT_POINTF commandVelocity;

    if( motionStep == 0 ) 
    {
        motionStep = 6;
    }

    // length wise motion
    if( motionStep == 1 )
    {
        if(lengthWiseMotion(nextPoint,lastPoint,commandVelocity,taskFinish))
        {
            motionStep = 5; 
        }
    }

    // transversal motion
    if( motionStep == 2 )
    {
        if(transversalMotion(nextPoint,lastPoint,commandVelocity,zoneMsg,taskFinish))
        {
            motionStep = 5;
        }
    }

    // from right side go to left side
    if( motionStep == 3 )
    {
        if(gotoNextSideZone(zoneMsg,zoneMap,commandVelocity,taskFinish))
        {
            if(taskFinish)
            {
                zoneMsg->arrive = true;
                zoneMsg->side = 2;
                motionStep = 0;
            }
            else
            {
                if(zoneMsg->num > 0)
                {
                    zoneMsg->num ++;
                }
                else if(zoneMsg->num <= 0)
                {
                    zoneMsg->num --;
                }

                zoneMsg->side = 0;
                zoneMsg->arrive = false;
                motionStep = 4;
            }
        }
    }

    // go to next zone
    if( motionStep == 4 )
    {
        //  go to next cut zone
        if(gotoNextZone(zoneMsg,zoneMap,commandVelocity))
        {
            if(zoneMsg->arrive)
            {
                free(zoneMap->data);
                initialGridMap(zoneMap);

                zoneMsg->mapNum = zoneMsg->num;

                zoneMsg->side = 1;
                motionStep = 6;
            }
            else
            {
                // motionStep = 5;
                if(zoneMsg->num > 0)
                {
                    zoneMsg->type = ( (zoneMsg->type + 1) % 3);

                    switch(zoneMsg->type)
                    {
                        case 1:
                            zoneMsg->yaw = sensorData.cutDirection;
                            break;
                        case 2:
                            zoneMsg->yaw = constrainAngle(sensorData.cutDirection + PI/2);
                            break;
                        default:
                            ROS_ERROR("error cut zone type!");
                    }

                    switch (zoneMsg->type)
                    {
                        case 0:
                            detectFinish = true;
                            motionStep = 0;
                            ROS_ERROR("random motion: finish the finial task.");
                            break;
                        case 1:
                        case 2:
                            sensorData.originPoint.x = robotPose.x;
                            sensorData.originPoint.y = robotPose.y;
                            motionStep = 0;
                            zoneMsg->side = 1;
                            zoneMsg->num = 0;
                            break;
                        default:
                            ROS_ERROR("error motion type !");
                    }
                }
                else if(zoneMsg->num <= 0)
                {
                    zoneMsg->num = 1;
                    zoneMsg->side = 0;
                    motionStep = 4;
                }
            }
        }
    }

    // find available yaw
    if( motionStep == 6 )
    {
        if(findEnableYaw(zoneMsg,commandVelocity))
        {
            sensorData.begainPoint.x = robotPose.x;
            sensorData.begainPoint.y = robotPose.y;

            float delta_AB = distanceCenterLine(zoneMsg,robotPoint);

            if(fabs(constrainAngle(robotPose.theta - zoneMsg->yaw - PI/2)) < 30 * DegreeToRad)
            {
                // follow the cut direction
                nextPoint.x = robotPose.x + (3 - delta_AB) * cos(zoneMsg->yaw + PI/2);
                nextPoint.y = robotPose.y + (3 - delta_AB) * sin(zoneMsg->yaw + PI/2);

                lastPoint.x = robotPose.x + (delta_AB + 3) * cos(zoneMsg->yaw + 3 * PI/2);
                lastPoint.y = robotPose.y + (delta_AB + 3) * sin(zoneMsg->yaw + 3 * PI/2);

                zoneMsg->alongAngle = constrainAngle(zoneMsg->yaw + 3*PI/2);

                motionSide = 0;
            }
            else
            {
                // follow the anti-cut direction
                nextPoint.x = robotPose.x + (delta_AB + 3.0) * cos(zoneMsg->yaw + 3 * PI/2);
                nextPoint.y = robotPose.y + (delta_AB + 3.0) * sin(zoneMsg->yaw + 3 * PI/2);

                lastPoint.x = robotPose.x + (3.0 - delta_AB) * cos(zoneMsg->yaw + PI/2);
                lastPoint.y = robotPose.y + (3.0 - delta_AB) * sin(zoneMsg->yaw + PI/2);

                zoneMsg->alongAngle = constrainAngle(zoneMsg->yaw + PI/2);

                motionSide = 2;
            }

            spdlog::get("robot_state")->info("the begain point is: x = {}, y = {}.",robotPose.x,robotPose.y);

            motionStep = 1; 

            if(zoneMsg->type == 0)
            {
                // finish the mission
                detectFinish = true;
                motionStep = 0;
            } 
        }
    }

    // go to remain zone
    if( motionStep == 7 )
    {
        if(gotoRemainZone(zoneMsg,zoneMap,commandVelocity,taskFinish))
        {
            if(taskFinish)
            {
                float delta_AB = distanceCenterLine(zoneMsg,sensorData.begainPoint);

                if(fabs(constrainAngle(robotPose.theta - zoneMsg->yaw + PI/2)) < 90 * DegreeToRad)
                {
                    nextPoint.x = robotPose.x + (3 + delta_AB) * cos(zoneMsg->yaw - PI/2);
                    nextPoint.y = robotPose.y + (3 + delta_AB) * sin(zoneMsg->yaw - PI/2);
                    motionSide = 2;
                }
                else
                {
                    nextPoint.x = robotPose.x + (3 - delta_AB) * cos(zoneMsg->yaw + PI/2);
                    nextPoint.y = robotPose.y + (3 - delta_AB) * sin(zoneMsg->yaw + PI/2);
                    motionSide = 0;
                }

                lastPoint.x = robotPose.x;
                lastPoint.y = robotPose.y;

                motionStep = 6;
            }
            else
            {
                motionStep = 8;
            }
        }
    }

    // manage path
    if( motionStep == 5 )
    {
        if(taskFinish)
        {
            if(findRemainZone(zoneMap,zoneMsg,breakPoint))
            {
                motionStep = 7;
            }
            else
            {
                if(zoneMsg->type == 0)
                {
                    // finish the mission
                    detectFinish = true;
                    motionStep = 0;
                }
                else
                {
                    motionStep = 8;
                }
            }
        }
        else
        {
            motionSide = (motionSide + 1) % 4;

            double pathDistance = sqrt(pow(nextPoint.x - lastPoint.x,2) + pow(nextPoint.y - lastPoint.y,2));

            YAT_POINTF pathVector;
            pathVector.x = (nextPoint.x - lastPoint.x) / pathDistance;
            pathVector.y = (nextPoint.y - lastPoint.y) / pathDistance;

            YAT_POINTF targetVector;
            targetVector.x = robotPose.x - lastPoint.x;
            targetVector.y = robotPose.y - lastPoint.y;

            float motionDistance = targetVector.x * pathVector.x + targetVector.y * pathVector.y;

            YAT_POINTF motionVector;
            motionVector.x = motionDistance * pathVector.x;
            motionVector.y = motionDistance * pathVector.y;

            if(motionSide % 2 == 0)
            {
                lastPoint = nextPoint;
            }
            else
            {
                lastPoint.x = lastPoint.x + motionVector.x;
                lastPoint.y = lastPoint.y + motionVector.y;
            }

            YAT_POINTF currentPoint;
            currentPoint.x = robotPose.x;
            currentPoint.y = robotPose.y;

            float delta_AB = distanceCenterLine(zoneMsg,currentPoint);

            switch (motionSide)
            {
                case 0:
                    nextPoint.x = lastPoint.x + fabs(3 - delta_AB) * cos(zoneMsg->yaw + PI/2);
                    nextPoint.y = lastPoint.y + fabs(3 - delta_AB) * sin(zoneMsg->yaw + PI/2);
                    motionStep = 1;
                    zoneMsg->alongAngle = constrainAngle( zoneMsg->yaw + PI/2 );
                    break;
                case 1:
                case 3:
                    if(zoneMsg->side == 1)
                    {
                        nextPoint.x = lastPoint.x + 0.2 * cos(zoneMsg->yaw);
                        nextPoint.y = lastPoint.y + 0.2 * sin(zoneMsg->yaw);
                    }
                    else
                    {
                        nextPoint.x = lastPoint.x + 0.2 * cos(zoneMsg->yaw + PI);
                        nextPoint.y = lastPoint.y + 0.2 * sin(zoneMsg->yaw + PI);
                    }
                    motionStep = 2;
                    break;
                case 2:
                    nextPoint.x = lastPoint.x - fabs(3 + delta_AB) * cos(zoneMsg->yaw + PI/2);
                    nextPoint.y = lastPoint.y - fabs(3 + delta_AB) * sin(zoneMsg->yaw + PI/2);
                    motionStep = 1;
                    zoneMsg->alongAngle = constrainAngle( zoneMsg->yaw + 3*PI/2 );
                    break;
                default:
                    break;
            }
        }

        commandVelocity.x = 0;
        commandVelocity.y = 0;
    }

    // manage zone
    if( motionStep == 8 )
    {
        switch (zoneMsg->side)
        {
            case 1:
                // from right side go to left side
                motionStep = 3;
                zoneMsg->side = 2;
                zoneMsg->arrive = false;
                break;
            case 2:
                // go to next zone
                motionStep = 4;
                zoneMsg->side = 0;
                zoneMsg->arrive = false;

                if(zoneMsg->num > 0)
                {
                    zoneMsg->num ++;
                }
                else if(zoneMsg->num <= 0)
                {
                    zoneMsg->num --;
                }

                break;
            default:
                ROS_ERROR("error cut zone side!");
                break;
        }
    }

    /************************************************/
    msg_vel.linear.x = commandVelocity.x;     // velocity control
    msg_vel.angular.z = commandVelocity.y;    // angle control

    spdlog::get("robot_state")->info("motion detect -> next point: x = {}, y = {} .",nextPoint.x,nextPoint.y);
    spdlog::get("robot_state")->info("motion detect -> last point: x = {}, y = {} .",lastPoint.x,lastPoint.y);
    spdlog::get("robot_state")->info("motion detect -> control msg: v = {}, w = {} .",commandVelocity.x,commandVelocity.y);
    spdlog::get("robot_state")->info("motion detect -> detector: left = {}, right = {} .",detectData.leftSide,detectData.rightSide);
    spdlog::get("robot_state")->info("motion detect -> cut zone: type = {}, num = {} .",zoneMsg->type,zoneMsg->num);
    spdlog::get("robot_state")->info("motion detect -> cut zone: side = {}, arrive = {} .",zoneMsg->side,zoneMsg->arrive);
    spdlog::get("robot_state")->info("motion detect step is : {}.\n",motionStep);

    vel_pub.publish(msg_vel);

    return detectFinish;
}

bool updateZoneMap(MAP_GRID *zoneMap,CutZoneType *zoneMsg)
{
   gridMapType parametre = getGridParametre(zoneMap);

    YAT_POINTF cutterCenter;
    cutterCenter.x = robotPose.x + 0.1 * cos(robotPose.theta);
    cutterCenter.y = robotPose.y + 0.1 * sin(robotPose.theta);

    // calcute the cutter point
    vector<YAT_POINTF> cutterPoint(20); // - 0.2 <= x <= 0.2;  0 <= y <= 0.2;
    vector<YAT_POINTF> cutterBody(20);
    for(int i = -2; i < 2; i++)
    {
        for(int j = -2; j < 3; j++)
        {
            double deltaXb = 0.1 * j;
            double deltaYb = 0.1 * i;
            cutterPoint[(i + 2) * 5 + (j + 2)].x = cutterCenter.x + deltaXb * sin(robotPose.theta) + deltaYb * cos(robotPose.theta);
            cutterPoint[(i + 2) * 5 + (j + 2)].y = cutterCenter.y - deltaXb * cos(robotPose.theta) + deltaYb * sin(robotPose.theta);

            cutterBody[(i + 2) * 5 + (j + 2)] = worldPointToMap(zoneMsg,cutterPoint[(i + 2) * 5 + (j + 2)]);
        }
    }

    // record the robot cutter point
    for(int k = 0; k < 20; k++)
    {
        YAT_POINT pointXY = worldToRaster(cutterBody[k],parametre);
        int i = pointXY.y;
        int j = pointXY.x;
        if(detectPointXY(parametre,pointXY))
        {
            if(zoneMap->data[i * parametre.rangeX + j] == ID_REMAIN)
            {
                zoneMap->data[i * parametre.rangeX + j] = ID_CUT;
            }
        }
        else
        {
            ROS_ERROR("invail grid point.");
        }
    }

    // calcute the bump point and record the bump point
    vector<YAT_POINTF> boundaryPoint(3);
    vector<YAT_POINTF> boundaryBody(3);
    for(int k = - 1; k <= 1; k++)
    {
        boundaryPoint[k + 1].x = robotPose.x + 0.05 * k * sin(robotPose.theta) + 0.4 * cos(robotPose.theta);
        boundaryPoint[k + 1].y = robotPose.y - 0.05 * k * cos(robotPose.theta) + 0.4 * sin(robotPose.theta);

        boundaryBody[k + 1] = worldPointToMap(zoneMsg,boundaryPoint[k + 1]);

        YAT_POINT pointXY = worldToRaster(boundaryBody[k + 1],parametre);
        int i = pointXY.y;
        int j = pointXY.x;

        if(detectPointXY(parametre,pointXY))
        {
            bool meetBump = (detectData.leftBump || detectData.rightBump);
            bool meetEdge = ((!detectData.leftSide) || (!detectData.rightSide));

            if(meetBump || meetEdge) zoneMap->data[i * parametre.rangeX + j] = ID_OBSTACLE; // 0 is enable; 100 is obstracle; - 1 is unknow 
        }
        else
        {
            ROS_ERROR("invail grid point.");
        }
    }
}

nav_msgs::OccupancyGrid postOccupancyMap(MAP_GRID *gridMap)
{
    gridMapType parametre = getGridParametre(gridMap);

    // ROS_INFO("Creating map rviz");
    nav_msgs::OccupancyGrid ocmap;
    ocmap.header.frame_id = "mowerMap";
    ocmap.header.stamp = ros::Time::now();

    ocmap.info.height = parametre.rangeY;
    ocmap.info.width = parametre.rangeX;
    ocmap.info.resolution = 1.0 / parametre.scale;
    ocmap.info.origin.position.x = parametre.offsetX;
    ocmap.info.origin.position.y = parametre.offsetY;

    ocmap.data.resize(ocmap.info.width * ocmap.info.height);

    for(int i = 0; i < ocmap.info.height; i++)
    {
        for(int j = 0; j < ocmap.info.width; j++)
        {
            ocmap.data[i * ocmap.info.width + j] = gridMap->data[i * ocmap.info.width + j];
        }
    }

    // ROS_INFO("ocmap data : hight = %d , width = %d .",ocmap.info.height, ocmap.info.width);

    return ocmap;
}

void odomCallback(geometry_msgs::PoseArray msg_alf)
{
    ros::Time currentTime=ros::Time::now();
    
    detectData.update = true;

    detectData.leftSide = (((int) msg_alf.poses[0].position.x) == 1)?true:false;
    detectData.rightSide = (((int) msg_alf.poses[0].position.y) == 1)?true:false;
    detectData.signalSide = (((int) msg_alf.poses[0].position.z) == 1)?true:false;

    detectData.leftValue = msg_alf.poses[1].position.x;
    detectData.rightValue = msg_alf.poses[1].position.y;
    detectData.chargeCurrent = msg_alf.poses[1].position.z;


    detectData.leftBump = (((int) msg_alf.poses[0].orientation.x) == 1)?true:false;
    detectData.rightBump = (((int) msg_alf.poses[0].orientation.y) == 1)?true:false;
    detectData.leftUp = (((int) msg_alf.poses[0].orientation.z) == 1)?true:false;
    detectData.rightUp = (((int) msg_alf.poses[0].orientation.w) == 1)?true:false;

    sensorData.updateBump = true;

    odomData.mil.x = msg_alf.poses[1].orientation.x; // left wheel mileage
    odomData.mil.y = msg_alf.poses[1].orientation.y; // right wheel mileage
    odomData.mil.z = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y); // mileage

    odomData.vel.x = msg_alf.poses[1].orientation.z; // left wheel velocity
    odomData.vel.y = msg_alf.poses[1].orientation.w; // right wheel velocity
    odomData.vel.z = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w); // velocity

    sensorData.updateEdge = true;
}

void ekfPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg_ekf)
{
    sensorData.updatePoseTime = ros::Time::now();

    // in the NEU navigation frame
    robotPose.x = msg_ekf->pose.position.x;
    robotPose.y = msg_ekf->pose.position.y;
    robotPose.theta = constrainAngle( msg_ekf->pose.orientation.z + PI/2);

    int rtkFlag = (int)msg_ekf->pose.orientation.w;

    spdlog::get("robot_state")->info("motion get robot pose is: x={}, y={}, theta={}.",robotPose.x, robotPose.y,robotPose.theta);
    // spdlog::get("robot_state")->info("------- The circuit Motion is starting ---------");
    sensorData.updatePose = true;
}

void gridMapCallback(nav_msgs::OccupancyGrid msg_map)
{
    sensorData.updateMapTime = ros::Time::now();

    sourceMap = msg_map;

    sensorData.updateMap = true;
    // ROS_INFO("get the grid map");
    spdlog::get("robot_state")->info("update the grid map from topic named grid_map.");
}

void gohomeCallback(std_msgs::Bool msg_gohome)
{
    if(msg_gohome.data)
    {
        sensorData.gobackCharging = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circuit");
    ROS_INFO_STREAM("circuit motion program is starting ...");

    ros::NodeHandle nh;

    auto my_logger = spdlog::rotating_logger_mt("robot_state", file_log, 1048576*20, 5);
    spdlog::get("robot_state")->info("------- The circuit Motion is starting ---------");

    ROS_INFO("error flag 0");

    // publish message
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    ros::Publisher occmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/zone_map", 1, true);

    // subscrible message
    ros::Subscriber pose_sub = nh.subscribe("/robot_pose", 1, ekfPoseCallback);
    ros::Subscriber map_sub = nh.subscribe("/grid_map", 1, gridMapCallback);
    ros::Subscriber odom_sub  = nh.subscribe("/alf001_dis", 1,odomCallback);
    ros::Subscriber gohome_sub  = nh.subscribe("/gohome", 1,gohomeCallback);

    MAP_GRID *currentMap = (MAP_GRID*)malloc(sizeof(MAP_GRID));
    CutZoneType *zoneData  = (CutZoneType*)malloc(sizeof(CutZoneType));

    srand(time(0));

    initializationMession(sensorData);

    bool initialMapRange = false;

    ros::Time updateEdgeTime = ros::Time::now();

    // ros::Rate loop_rate(10); 
    while(ros::ok())
    {
        if(initialMapRange)
        {
            if(sensorData.updateEdge) updateEdgeTime = ros::Time::now();

            if( (!sensorData.updatePose) || (!sensorData.updateBump) || (!sensorData.updateEdge))
            {
                float lostEdgeTime = ros::Time::now().toSec() - updateEdgeTime.toSec();

                if( fabs(lostEdgeTime) > 0.5 )
                {
                    msg_vel.linear.x = 0;     // velocity control
                    msg_vel.angular.z = 0;    // angle control

                    vel_pub.publish(msg_vel);
                }

                // if(!sensorData.updateMap) ROS_INFO("lost map data.");
                if(!sensorData.updatePose) ROS_INFO("lost pose data.");
                if(!sensorData.updateBump) ROS_INFO("lost bump data.");
                if(!sensorData.updateEdge) ROS_INFO("lost edge data.");
                // if(!sensorData.updateStuck) ROS_INFO("lost stuck data.");
            }
            else
            {
                updateZoneMap(currentMap,zoneData);

                nav_msgs::OccupancyGrid postMap = postOccupancyMap(currentMap);
                occmap_pub.publish(postMap);

                if(sensorData.gobackCharging)
                {
                    if(gobackHome())
                    {
                        sensorData.finishTask = true;
                        break;
                    }
                }
                else
                {
                    if(motionDetect(currentMap,zoneData))
                    {
                        // sensorData.finishTask = true;
                        ROS_INFO("zigzag motion has finished, then go back for charge.");
                        sensorData.gobackCharging = true;
                    }
                }
            }
        }
        else
        {   
            // ROS_INFO("wait for robot pose data.");
            if(sensorData.updatePose )// && sensorData.updateMap
            {
                sensorData.originPoint.x = robotPose.x;
                sensorData.originPoint.y = robotPose.y;

                zoneData->num = 0;
                zoneData->type = 1;
                zoneData->side = 1;
                zoneData->arrive = true;
                zoneData->yaw = sensorData.cutDirection;
                zoneData->mapNum = zoneData->num;

                ROS_INFO("initial the zone map : flag 0.1.");

                initialGridMap(currentMap);

                ROS_INFO("initial the zone map : flag 0.2");

                initialMapRange = true;
            }
        }

        initializationSensor(sensorData);

        ros::spinOnce();
        // loop_rate.sleep();
        ros::Duration(0.1).sleep();
    }

    fout_point.close();
    return 0;
}