//ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
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
#include "std_msgs/String.h"

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

#define filterYawNum 1000


typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                        geometry_msgs::PoseStamped> sync_policy_classifiction;

//get the current date //date -s "20180929 08:00:00"
std::string getDate();

//map point file
std::string point_path = "/home/rasp008/mower_ws/src/circuit_zigzag/data/";
std::string file_status = point_path + getDate() + "sensor0.txt";
std::ofstream fout_point(file_status.c_str());

//yaw data file
std::string file_yaw = point_path + getDate() + "initialYaw.txt";
std::ofstream fout_yawdata(file_yaw.c_str());

typedef struct
{
    bool update;

    POINT_XYZ baseBLH;  // base position from GPS in BLH frame
    POINT_XYZ baseECEF; // base position from GPS in ECEF frame

    POINT_XYZ pointBLH;  // real-time position from GPS in BLH frame
    POINT_XYZ pointECEF; // real-time position from GPS in ECEF frame

    double velocity; // velocity from GPS
    double direction; // direction from GPS
}GNSS_TYPE; 

typedef struct
{
    bool update;

    POINT_XYZ euler;
    POINT_XYZ acc;  // base position from GPS in BLH frame
    POINT_XYZ gyro; // base position from GPS in ECEF frame

    POINT_XYZ bg;  // real-time position from GPS in BLH frame
    POINT_XYZ ba;  // real-time position from GPS in ECEF frame
}IMU_TYPE;

typedef struct WORLD_POINTF_THETA
{
	double  x; // coordinate in X axis
	double  y; // coordinate in Y axis
	double  theta; // eular angle in z axis
}YAT_POSE;

typedef struct Boundary_Point
{
    bool feature;

    int id; // identification number

    double totalMile; // total mileage from origin point

    bool leftInside; // the left detector is inside;
    bool rightInside; // the right detector is inside;

    YAT_POSE pose; // boundary coordinates
    
    Boundary_Point *next;
}MAP_BOUNDARY_TYPE;

typedef struct Boundary_Feature
{
    int id; // identification number

    int num;

    bool inside; 

    double mileage;
    double ratio;

    YAT_POINTF point;
    YAT_POINTF theta;

    double covarXX;
    double covarXY;
    double covarYX;
    double covarYY;
    
    YAT_POINTF offset;

    Boundary_Feature *next;
}MAP_FEATURE_TYPE;

typedef struct
{
    int num;

    int id;

    double theta; // angle offset -> yaw in begain point

    double x; // X axis offset -> X bias position relative to origin point after a circle
    double y; // Y axis offset -> Y bias position relative to origin point after a circle
}BOUNDARY_CALIBRATION_TYPE;

typedef struct mapType
{
    int scale;

    int rangeX;
	int rangeY;

	double offsetX;
	double offsetY;

    int *data;
}MAP_GRID;

typedef struct
{
    bool getYawOffset;
    bool motionFinish;
    bool initialFinish;
    bool alongFinish;
    bool recordBoundary;
}MISSION_FLAG;

/******************* ros publisher ****************************/
ros::Publisher vel_pub;
ros::Publisher initial_pub;
ros::Publisher boundary_pub;

DETECT_TYPE detectData;
IMU_TYPE imuData;
ODOM_TYPE odomData;
GNSS_TYPE gnssData;

CONFIG_PARAMETER configData;
SOLUTION_PARAMETER solutionData;

BOUNDARY_CALIBRATION_TYPE initialData;
BOUNDARY_CALIBRATION_TYPE initialLast;

MISSION_FLAG missionSymbol;

geometry_msgs::Twist targetStates;
POINT_XYZ robotPose;
YAT_POSE mowerPose;

Boundary_Point *rawBoundaryPoint = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
Boundary_Feature *rawFeaturePoint = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));

Boundary_Point *correctBoundary = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
Boundary_Feature *correctFeature = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));

Boundary_Point *filterBoundary = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
Boundary_Feature *filterFeature = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));

Boundary_Point *lastBoundary = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
Boundary_Feature *lastFeature = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));

/*******************************************************************/
YAT_POSE getRealTimePosition()
{
    static ros::Time startTime = ros::Time::now();
    double deltaTime = ros::Time::now().toSec() - startTime.toSec();
    startTime = ros::Time::now();

    double currentMileage = configData.odomMil.z;

    YAT_POSE mowerData = mowerPose;
    static double mileage_prev = currentMileage;

    double deltaTheta = deltaTime * (configData.gyro.z - configData.gyroBias.z);
    double deltaMileage = currentMileage - mileage_prev; // delta yaw measured by imu
    mileage_prev = currentMileage;

    mowerData.x = mowerData.x + deltaMileage * cos(mowerData.theta + 0.5 * deltaTheta);
    mowerData.y = mowerData.y + deltaMileage * sin(mowerData.theta + 0.5 * deltaTheta);
    mowerData.theta = constrainAngle(mowerData.theta + deltaTheta);

    return mowerData;
}

void recordBoundaryData(bool feutureFlag)
{
    // record the boundary point in 'edgePoint' 
    Boundary_Point *newPointer = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
    newPointer->next = NULL;
    
    if(newPointer == NULL)
    {
        ROS_ERROR("memory allocation for a new node pointer is failure.");
        return;
    }

    Boundary_Point *sp = rawBoundaryPoint; // struct pointer

    int edgeNum = 0;
    while(sp->next != NULL)  // guarantee the pointer of edgePoint is NULL
    {
        edgeNum ++;
        sp = sp->next;
    }

    sp->id = edgeNum + 1;
    sp->feature = feutureFlag;
    sp->totalMile = configData.odomMil.z;
    sp->leftInside = detectData.leftSide;
    sp->rightInside = detectData.rightSide;
    sp->pose = mowerPose;
    sp->next = newPointer;
}

YAT_POINT worldToRaster(YAT_POINTF mapPoint,MAP_GRID *workMap)
{
    YAT_POINT gridPoint;
    gridPoint.x = (int)((mapPoint.x - workMap->offsetX) * workMap->scale + 0.5);
    gridPoint.y = (int)((mapPoint.y - workMap->offsetY) * workMap->scale + 0.5); 
    return gridPoint;
}

YAT_POINTF rasterToWorld(YAT_POINT gridPoint,MAP_GRID *workMap)
{
    YAT_POINTF mapPoint;
    mapPoint.x = ((double)gridPoint.x )/workMap->scale + workMap->offsetX;
    mapPoint.y = ((double)gridPoint.y )/workMap->scale + workMap->offsetY;
    return mapPoint;
}

bool detectPointXY(MAP_GRID *workMap,YAT_POINT detectePoint)
{   
    if( detectePoint.x < 0 )
    {
        return false;
    }

    if( detectePoint.x > workMap->rangeX - 1 )
    {
        return false;
    }

    if( detectePoint.y < 0 )
    {
        return false;
    }

    if( detectePoint.y > workMap->rangeY - 1 )
    {
        return false;
    }

    return true;
}

void markBoundaryPoint(MAP_GRID *workMap,YAT_POSE nextPose,YAT_POSE lastPose)
{
    int departNum = (int)(sqrt(pow(nextPose.x - lastPose.x,2) + pow(nextPose.y - lastPose.y,2)) * workMap->scale * 2  + 1);

    for(int i = 0; i < departNum ; i++)
    {
        YAT_POINTF densePoint;
        densePoint.x = ( (departNum - i) * lastPose.x + i * nextPose.x)/departNum;
        densePoint.y = ( (departNum - i) * lastPose.y + i * nextPose.y)/departNum;

        YAT_POINT denseXY = worldToRaster(densePoint,workMap);
        if(detectPointXY(workMap,denseXY))
        {
            workMap->data[denseXY.y * workMap->rangeX + denseXY.x] = 100;
        }
        else
        {
            ROS_ERROR("error grid map parametre!");
        }
    }
}

void initialGridMap(MAP_GRID *workMap)
{
    Boundary_Point *sp = filterBoundary;
    Boundary_Point *spA = filterBoundary;

    YAT_POINTF maxPoint;
    YAT_POINTF minPoint;

    if(sp->next == NULL)
    {
        ROS_ERROR("did not record the boundary data.");
        return;
    }
    else
    {
        maxPoint.x = sp->pose.x;
        maxPoint.y = sp->pose.y;

        minPoint.x = sp->pose.x;
        minPoint.y = sp->pose.y;
    }

    while(sp->next != NULL)  // guarantee the pointer of edgePoint is NULL
    {
        if(sp->pose.x < minPoint.x)
        {
            minPoint.x = sp->pose.x;
        }

        if(sp->pose.y < minPoint.y)
        {
            minPoint.y = sp->pose.y;
        }

        if(sp->pose.x > maxPoint.x)
        {
            maxPoint.x = sp->pose.x;
        }

        if(sp->pose.y > maxPoint.y)
        {
            maxPoint.y = sp->pose.y;
        }

        sp = sp->next;
    }

    double remainWidth = 2.0;

    workMap->scale = 10;

    workMap->rangeX = (int)( (maxPoint.x - minPoint.x + 2 * remainWidth) * workMap->scale ) + 1;
    workMap->rangeY = (int)( (maxPoint.y - minPoint.y + 2 * remainWidth) * workMap->scale ) + 1;
    workMap->offsetX = minPoint.x - remainWidth;
    workMap->offsetY = minPoint.y - remainWidth;

    ROS_INFO("grid map : width = %d, height = %d.",workMap->rangeX,workMap->rangeY);

    workMap->data = (int *)malloc(workMap->rangeX * workMap->rangeY * sizeof(int));

    if (!workMap->data)
    {
        ROS_ERROR("can not create map data");
        return;
    }

    // initial the total grid map
    for(int i = 0; i < workMap->rangeY; i++)
    {
        for(int j = 0; j < workMap->rangeX; j++)
        {
            workMap->data[i * workMap->rangeX + j] = 50;
        }
    }

    bool getStartPose = false;
    YAT_POSE startPose;
    YAT_POSE lastPose;

    while(spA->next != NULL)  // guarantee the pointer of edgePoint is NULL
    {
        if(getStartPose)
        {
            markBoundaryPoint(workMap,spA->pose,lastPose);
        }
        else
        {
            startPose = spA->pose;
            getStartPose = true;
        }

        lastPose = spA->pose;

        spA = spA->next;
    }

    // if(getStartPose) markBoundaryPoint(workMap,startPose,lastPose);
}

bool getMatchPoint(Boundary_Feature *lfB)
{
    bool matchResult = false;

    ROS_INFO("initialization get feature flag 5.1");

    Boundary_Feature *lfA = lastFeature;
    Boundary_Feature *lfC = lastFeature;

    double minAngle = 3 * 360; // degree

    ROS_INFO("detect NO.%d sfD",lfB->id);

    while(lfA->next != NULL)
    {
        bool meetRatio = (fabs(lfA->ratio - lfB->ratio) < 0.05)?true:false;

        if((lfA->inside == lfB->inside) && meetRatio)
        {
            double deltaThetaA = constrainAngle( lfA->theta.x - lfB->theta.x) * RadToDegree;
            double deltaThetaB = constrainAngle( lfA->theta.y - lfB->theta.y) * RadToDegree;

            bool meetAngleA = (fabs(deltaThetaA) < 30)?true:false;
            bool meetAngleB = (fabs(deltaThetaB) < 30)?true:false;

            // double deltaPosition = sqrt(pow(lfB->point.x,2) + pow(lfB->point.y,2)); // the straight distance from start point

            // bool meetPointA = (fabs(lfA->point.x - lfB->point.x) < 0.0875 * deltaPosition)?true:false;  // angle error is 5 degree : 0.0875 = tand(5) 
            // bool meetPointB = (fabs(lfA->point.y - lfB->point.y) < 0.0875 * deltaPosition)?true:false;
            
            // bool meetMileage = (fabs(lfA->mileage - lfB->mileage) < 0.1 * lfA->mileage)?true:false;

            double deltaThetaC = constrainAngle( lfA->theta.y - lfA->theta.x);
            double deltaThetaD = constrainAngle( lfB->theta.y - lfB->theta.x);
            double deltaThetaE = constrainAngle( deltaThetaC - deltaThetaD) * RadToDegree;

            bool meetDelta = (fabs(deltaThetaE) < 10)?true:false;

            ROS_INFO("detect NO.%d lfA data: A = %f, B = %f, ratio = %f, delta = %f",lfA->id,deltaThetaA,deltaThetaB,lfA->ratio - lfB->ratio,deltaThetaE);

            if(meetAngleA && meetAngleB  && meetDelta) //  && meetMileage
            {
                double functionValue = fabs(deltaThetaA) + fabs(deltaThetaB) + 2 * fabs(deltaThetaE);  // empirical function

                if(minAngle > functionValue)
                {
                    lfC = lfA;
                    minAngle = functionValue;
                    matchResult = true;
                }
            }
        }

        lfA = lfA->next;
    }

    if(matchResult) 
    {
        lfB->covarXX = lfC->covarXX;
        lfB->covarXY = lfC->covarXY;
        lfB->covarYX = lfC->covarYX;
        lfB->covarYY = lfC->covarYY; 

        lfB->point.x = lfC->point.x; 
        lfB->point.y = lfC->point.y; 

        ROS_INFO("detect data:  NO.%d sfD sames NO.%d lfA data",lfB->id,lfC->id);
    }

    return matchResult;
}

void fileCopy(string *file_r,string *file_w)
{
    ROS_INFO("Map copied doing.\n");
    FILE *op,*inp;
    op = fopen(file_r->c_str(),"rb");
    inp = fopen(file_w->c_str(),"wb");

    void *buf;

    while(!feof(op))
    {
        fread(&buf,1,1,op);
        fwrite(&buf,1,1,inp);
    }

    fclose(inp);
    fclose(op);
    ROS_INFO("Map copied done.\n");
}

bool getBoundaryPoint(MAP_GRID *workMap)
{
    // boundary
    std::string boundary_path = "/home/rasp008/mower_ws/src/circuit_zigzag/data/boundary/";
    // std::string boundary_path = "/home/yat/catkin_ws/src/circuit_zigzag/data/boundary/";
    std::string file_boundary = boundary_path + getDate() +"boundary.txt";
    std::string save_boundary = boundary_path + "boundary.txt";
    std::ofstream fout_boundary(file_boundary.c_str(),ios::out);

    Boundary_Point *sp = rawBoundaryPoint; // struct pointer

    YAT_POSE pointA;
    YAT_POSE pointB;

    YAT_POINTF mileData;

    Boundary_Feature *sf = rawFeaturePoint;

    bool lastFlag = false;
    int fatureNum = 0;
    int fatureID = 0;
    bool fatureInSide = false;
    
    YAT_POINTF featurePoint;
    YAT_POINTF featureTheta;

    if(sp == NULL)
    {
        ROS_ERROR("did not record the boundary data.");
        return false;
    }
    else
    {
        pointA = sp->pose;

        mileData.x = sp->totalMile;

        lastFlag = sp->feature;

        featurePoint.x = 0;
        featurePoint.y = 0;
    }

    fout_boundary.setf(std::ios_base::showpoint);
    fout_boundary.precision(15);

    int edgeNum = 0;
    while(sp->next != NULL)  // guarantee the pointer of edgePoint is NULL
    {
        if(sp->feature || lastFlag)
        {
            featurePoint.x = (fatureNum * featurePoint.x + sp->pose.x)/(fatureNum + 1);
            featurePoint.y = (fatureNum * featurePoint.y + sp->pose.y)/(fatureNum + 1);
            fatureNum++;

            if(sp->feature && (!lastFlag)) 
            {
                fatureInSide = sp->leftInside;
                featureTheta.x = sp->pose.theta;
            }            
            else if((!sp->feature) && lastFlag)
            {
                featureTheta.y = sp->pose.theta;

                double deltaTheta = constrainAngle(featureTheta.y - featureTheta.x);

                if(fabs(deltaTheta) > 0.25 * PI)
                {
                    Boundary_Feature *newPointer = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));
                    newPointer->next = NULL; 

                    fatureID++;
                    sf->id = fatureID; // identification number
                    sf->num = sp->id;
                    sf->inside = fatureInSide; 
                    sf->point = featurePoint;
                    sf->theta = featureTheta;
                    sf->mileage = sp->totalMile - mileData.x;
                    sf->ratio = 0;

                    sf->next = newPointer;
                    
                    fout_boundary << " 2 " << " " << sf->id << " " << sf->mileage  << " ";
                    fout_boundary << sf->inside << " " << sf->point.x << " " << sf->point.y << " ";
                    fout_boundary << sf->theta.x << " " << sf->theta.y << " " << sf->num  << " ";
                    fout_boundary << sf->ratio << " " << " 0 " << " " << " 0 " << std::endl;

                    sf = sf->next;
                }

                fatureNum = 0;
                featurePoint.x = 0;
                featurePoint.y = 0;
            }

            lastFlag = sp->feature;
        }

        pointB = sp->pose;
        
        edgeNum ++;

        mileData.y = sp->totalMile;

        fout_boundary << " 1 " << " " << sp->id << " " << sp->totalMile  << " ";
        fout_boundary << sp->feature << " " << sp->leftInside << " " << sp->rightInside << " ";
        fout_boundary << sp->pose.x << " " << sp->pose.y << " " << sp->pose.theta  << " ";
        fout_boundary << " 0 " << " " << " 0 " << " " << " 0 " << std::endl;

        sp = sp->next;
    }

    YAT_POINTF pointC;
    pointC.x = pointA.x  + 1.0 * cos(pointA.theta);
    pointC.y = pointA.y  + 1.0 * sin(pointA.theta);

    bool meetPosition = ( sqrt(pow(pointA.x - pointC.x,2) + pow(pointA.y - pointC.y,2) ) < 0.1 * fabs(mileData.y - mileData.x) )?true:false; 
    bool meetMileage = (fabs(mileData.y - mileData.x) > 10.0)?true:false;

    bool circuitFinish = ((meetPosition && meetMileage) || missionSymbol.getYawOffset);

    initialData.num = edgeNum;

    initialData.x = pointC.x - pointB.x;
    initialData.y = pointC.y - pointB.y;

    double rotateAngle = initialData.theta;
    double angleOffset = constrainAngle(initialLast.theta + initialData.theta);

    fout_boundary << " 0 " << " " << initialData.num << " " << circuitFinish  << " ";
    fout_boundary << initialData.x << " " << initialData.y << " " << initialData.theta << " ";
    fout_boundary << mileData.x << " " << mileData.y << " " << angleOffset  << " ";
    fout_boundary << missionSymbol.getYawOffset << " " << initialLast.theta << " " << " 0 " << std::endl;

    bool taskFinish = false;

    if(meetPosition && meetMileage)
    {
        Boundary_Point *spA = rawBoundaryPoint; // struct pointer
        Boundary_Point *spB = correctBoundary; // struct pointer

        while(spA->next != NULL)  // guarantee the pointer of edgePoint is NULL
        {
            double lambda = fabs(spA->totalMile - mileData.x) / fabs(mileData.y - mileData.x);

            YAT_POINTF correctPosition;
            correctPosition.x = spA->pose.x + lambda * initialData.x;
            correctPosition.y = spA->pose.y + lambda * initialData.y;

            YAT_POSE worldPose;
            worldPose.x = correctPosition.x * cos(rotateAngle) - correctPosition.y * sin(rotateAngle);
            worldPose.y = correctPosition.x * sin(rotateAngle) + correctPosition.y * cos(rotateAngle);
            worldPose.theta = constrainAngle(spA->pose.theta + rotateAngle);

            Boundary_Point *newPointer = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
            newPointer->next = NULL;

            spB->id = spA->id;
            spB->feature = spA->feature;
            spB->totalMile = spA->totalMile;
            spB->leftInside = spA->leftInside;
            spB->rightInside = detectData.rightSide;
            spB->pose = worldPose;
            spB->next = newPointer;

            fout_boundary << " 3 " << " " << spB->id << " " << spB->totalMile  << " ";
            fout_boundary << spB->feature << " " << spB->leftInside << " " << spB->rightInside << " ";
            fout_boundary << spB->pose.x << " " << spB->pose.y << " " << spB->pose.theta  << " ";
            fout_boundary << " 0 " << " " << " 0 " << " " << " 0 " << std::endl;

            spB = spB->next;
            spA = spA->next;
        }

        Boundary_Feature *sfA = rawFeaturePoint;
        Boundary_Feature *sfB = correctFeature;

        while(sfA->next != NULL)  // guarantee the pointer of edgePoint is NULL
        {
            double lambda = fabs(sfA->mileage) / fabs(mileData.y - mileData.x);

            YAT_POINTF correctPosition;
            correctPosition.x = sfA->point.x + lambda * initialData.x;
            correctPosition.y = sfA->point.y + lambda * initialData.y;

            YAT_POINTF worldPoint;
            worldPoint.x = correctPosition.x * cos(rotateAngle) - correctPosition.y * sin(rotateAngle);
            worldPoint.y = correctPosition.x * sin(rotateAngle) + correctPosition.y * cos(rotateAngle);

            Boundary_Feature *newPointer = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));
            newPointer->next = NULL; 

            sfB->id = sfA->id;
            sfB->num = sfA->num;
            sfB->inside = sfA->inside; 
            sfB->point = worldPoint;
            sfB->theta.x = constrainAngle(sfA->theta.x + rotateAngle);
            sfB->theta.y = constrainAngle(sfA->theta.y + rotateAngle);
            sfB->mileage = sfA->mileage;
            sfB->ratio = lambda;

            sfB->next = newPointer;

            fout_boundary << " 4 " << " " << sfB->id << " " << sfB->mileage  << " ";
            fout_boundary << sfB->inside << " " << sfB->point.x << " " << sfB->point.y << " ";
            fout_boundary << sfB->theta.x << " " << sfB->theta.y << " " << sfB->num  << " ";
            fout_boundary << sfB->ratio << " " << " 0 " << " " << " 0 " << std::endl;

            sfB = sfB->next;
            sfA = sfA->next;
        }

        Boundary_Feature *sfC = correctFeature;
        Boundary_Feature *sfK = filterFeature;

        if(missionSymbol.getYawOffset)
        {
            while(sfC->next != NULL)  // guarantee the pointer of edgePoint is NULL
            {
                Boundary_Feature *sfD = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));;
                sfD->id = sfC->id;
                sfD->inside = sfC->inside;
                sfD->theta.x = sfC->theta.x;
                sfD->theta.y = sfC->theta.y;
                sfD->ratio = sfC->ratio;
                sfD->next = NULL;

                if(getMatchPoint(sfD))
                {
                    // the feature filter : sfC is observation ; sfD is last filter result; sfK is current filter result;
                    Eigen::Matrix<double,2,2> PHI;
                    PHI.setZero(2,2);
                    PHI(0,0) = 1;
                    PHI(1,1) = 1;

                    Eigen::Matrix<double,2,2> P0;
                    P0.setZero(2,2);
                    P0(0,0) = sfD->covarXX;
                    P0(0,1) = sfD->covarXY;
                    P0(1,0) = sfD->covarYX;
                    P0(1,1) = sfD->covarYY;
                    
                    ROS_INFO("NO.%d sfD covar data is : xx = %f, xy = %f, yx = %f, yy = %f.",sfC->id,sfD->covarXX,sfD->covarXY,sfD->covarYX,sfD->covarYY);

                    // configure system state Noise covariance Q
                    Eigen::Matrix<double,2,2> Q;
                    Q.setZero(2,2);

                    // update Prior estimate covariance
                    Eigen::Matrix<double,2,2> P_p = PHI * P0 * PHI.transpose() + Q;

                    // configure Observation matrix
                    Eigen::Matrix<double,2,2> H;
                    H.setZero(2,2);
                    H(0,0) = 1;
                    H(1,1) = 1;

                    // configure Observation noise covariance R
                    Eigen::Matrix<double,2,2> R;
                    R.setZero(2,2);
                    R(0,0) = pow(initialData.x,2); //  * sfC->ratio
                    R(1,1) = pow(initialData.y,2); //  * sfC->ratio

                    // kalman filter gain
                    Eigen::Matrix<double,2,2> K = P_p*H.transpose()*( H*P_p*H.transpose() + R).inverse();

                    ROS_INFO("kalman gain data is : xx = %f, xy = %f, yx = %f, yy = %f.",K(0,0),K(0,1),K(1,0),K(1,1));

                    // configure unit matrix
                    Eigen::Matrix<double,2,2> I;
                    I.setIdentity(2,2);

                    // update posteriori estimate covariance
                    Eigen::Matrix<double,2,2> P_c = (I - K*H)*P_p;
                    sfK->covarXX = P_c(0,0);
                    sfK->covarXY = P_c(0,1);
                    sfK->covarYX = P_c(1,0);
                    sfK->covarYY = P_c(1,1);

                    // update Observation vector
                    Eigen::Matrix<double,2,1> Y;
                    Y(0,0) = sfC->point.x - sfD->point.x;
                    Y(1,0) = sfC->point.y - sfD->point.y;

                    // update posteriori evaluation states
                    Eigen::Matrix<double,2,1> X_f = K * Y;
                    
                    YAT_POINTF filterResult;
                    filterResult.x = sfD->point.x + X_f(0,0);
                    filterResult.y = sfD->point.y + X_f(1,0);

                    Boundary_Feature *newPointer = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));
                    newPointer->next = NULL; 

                    sfK->id = sfC->id;
                    sfK->num = sfC->num;
                    sfK->inside = sfC->inside; 
                    sfK->point = filterResult;
                    sfK->theta.x = sfC->theta.x;
                    sfK->theta.y = sfC->theta.y;
                    sfK->mileage = sfC->mileage;
                    sfK->ratio = sfC->ratio;

                    sfK->offset.x = filterResult.x - sfC->point.x;
                    sfK->offset.y = filterResult.y - sfC->point.y;

                    ROS_INFO("the NO.%d point offset is: x = %f, y = %f .",sfK->id,sfK->offset.x,sfK->offset.y);

                    sfK->next = newPointer;

                    fout_boundary << " 6 " << " " << sfK->id << " " << sfK->point.x << " ";
                    fout_boundary << sfK->point.y << " " << sfK->theta.x << " " << sfK->theta.y << " ";
                    fout_boundary << sfK->inside << " " << sfK->ratio  << " " << sfK->covarXX << " ";
                    fout_boundary << sfK->covarXY << " " << sfK->covarYX << " " << sfK->covarYY << std::endl;

                    sfK = sfK->next;
                }
                else
                {
                    // add the Obvious features
                }

                free(sfD);

                sfC = sfC->next;
            }
        }
        else
        {
            filterFeature = correctFeature;
            
            sfK = filterFeature;
            
            while(sfK->next != NULL)
            {
                sfK->offset.x = 0;
                sfK->offset.y = 0;

                sfK->covarXX = pow(initialData.x,2);
                sfK->covarXY = 0;
                sfK->covarYX = 0;
                sfK->covarYY = pow(initialData.y,2);

                fout_boundary << " 6 " << " " << sfK->id << " " << sfK->point.x << " ";
                fout_boundary << sfK->point.y << " " << sfK->theta.x << " " << sfK->theta.y << " ";
                fout_boundary << sfK->inside << " " << sfK->ratio  << " " << sfK->covarXX << " ";
                fout_boundary << sfK->covarXY << " " << sfK->covarYX << " " << sfK->covarYY << std::endl;

                sfK = sfK->next;
            }
        }

        Boundary_Point *spC = correctBoundary;
        Boundary_Point *spD = filterBoundary;

        Boundary_Feature *sfE = filterFeature;

        YAT_POINTF passMileage;

        YAT_POINTF thisOffset;

        YAT_POINTF lastOffset;
        lastOffset.x = 0;
        lastOffset.y = 0;

        if(sfE->next == NULL)
        {
            passMileage.x = mileData.x;
            passMileage.y = mileData.y;

            thisOffset.x = 0;
            thisOffset.y = 0;
        }
        else
        {
            passMileage.x = mileData.x;
            passMileage.y = mileData.x + sfE->mileage;

            thisOffset = sfE->offset;
        }

        while(spC->next != NULL)
        {
            if(sfE->next != NULL)
            {
                if(spC->id == sfE->num)
                {
                    sfE = sfE->next;

                    lastOffset = thisOffset;

                    if(sfE->next != NULL)
                    {
                        thisOffset = sfE->offset;
                        passMileage.x = spC->totalMile;
                        passMileage.y = mileData.x + sfE->mileage;
                    }
                    else
                    {
                        // the final feature
                        thisOffset.x = 0;
                        thisOffset.x = 0;

                        passMileage.x = mileData.y;
                        passMileage.y = passMileage.y;
                    }
                }
            }

            double lambda = fabs(spC->totalMile - passMileage.x) / fabs(passMileage.y - passMileage.x);

            Boundary_Point *newPointer = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
            newPointer->next = NULL;

            spD->id = spC->id;
            spD->feature = spC->feature;
            spD->totalMile = spC->totalMile;
            spD->leftInside = spC->leftInside;
            spD->rightInside =spC->rightInside;
            spD->pose.x = spC->pose.x + (1 - lambda) * lastOffset.x + lambda * thisOffset.x;
            spD->pose.y = spC->pose.y + (1 - lambda) * lastOffset.y + lambda * thisOffset.y;
            spD->pose.theta =  spC->pose.theta;
            spD->next = newPointer;

            fout_boundary << " 5 " << " " << spD->id << " " << spD->totalMile  << " ";
            fout_boundary << spD->pose.x << " " << spD->pose.y << " " << spD->pose.theta << " ";
            fout_boundary << lambda << " " << thisOffset.x << " " << thisOffset.y  << " ";
            fout_boundary << " 0 " << " " << " 0 " << " " << " 0 " << std::endl;

            spD = spD->next;
            spC = spC->next;
        }

        initialGridMap(workMap);

        taskFinish = true;
    }
    else
    {
        ROS_ERROR("did not get a circle boundary data.");
    }

    fout_boundary.close();

    fileCopy(&file_boundary,&save_boundary);

    return taskFinish;
}

void locationDR()
{
    double currentYaw = configData.euler.z;
    double currentMileage = configData.odomMil.z;

    static double yaw_prev = currentYaw; 
    static double mileage_prev = currentMileage; 
    static Eigen::Vector3d PositionDR(0,0,PI/2); // PI/2

    double deltaMileage = currentMileage - mileage_prev; // delta yaw measured by imu
    double deltaYaw = deltaAngle(currentYaw,yaw_prev); // current mileage measured by odom

    /***********************************************  evaluate PositionDR by IMU and mileage *************************************************/ 
    Eigen::Vector3d deltaPosition(deltaMileage*cos(PositionDR(2)+deltaYaw/2),deltaMileage*sin(PositionDR(2)+deltaYaw/2),deltaYaw);
    PositionDR = PositionDR + deltaPosition;
    /***********************************************************************************************************************************/ 
    PositionDR(2) = constrainAngle(PositionDR(2));

    // ROS_INFO(" robot position is: x = %f , y = %f, theta = %f .",PositionDR(0),PositionDR(1),PositionDR(2)*180/PI);    

    robotPose.x = PositionDR(0);
    robotPose.y = PositionDR(1);
    robotPose.z = PositionDR(2);

    //update the prev data
    yaw_prev = currentYaw; 
    mileage_prev = currentMileage;
}

double deltaDistance(POINT_XYZ pointA, POINT_XYZ pointB)
{
    double delta_X = pointA.x - pointB.x;
    double delta_Y = pointA.y - pointB.y;

    return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

double distanceToAB(POINT_XYZ pointA, POINT_XYZ pointB,POINT_XYZ currentPoint)
{
    // angle control
    double pathDistance = deltaDistance(pointA , pointB); 

    YAT_POINTF pathVector,normalVector,targetVector;
    pathVector.x = (pointA.x - pointB.x)/pathDistance;
    pathVector.y = (pointA.y - pointB.y)/pathDistance;

    // clockwise vector
    normalVector.x = pathVector.y;
    normalVector.y = - pathVector.x;

    targetVector.x = pointA.x - currentPoint.x;
    targetVector.y = pointA.y - currentPoint.y;

    return (targetVector.x*normalVector.x + targetVector.y*normalVector.y);
}

int sign(double data)
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

bool straightMotion(POINT_XYZ nextPoint,POINT_XYZ lastPoint)
{
    // angle control
    double pathDistance = deltaDistance(nextPoint , lastPoint); 

    Eigen::Vector2d pathVector( (nextPoint.x - lastPoint.x)/pathDistance, (nextPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(nextPoint.x - robotPose.x, nextPoint.y - robotPose.y);

    double trajectoryDistance = distanceToAB(nextPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double PathAngle = atan2( nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double aimLength = 0.3;
 
    double controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength)
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    double targetRobotPose = constrainAngle(controlAngle + PathAngle);
    double  controlObject = deltaAngle(targetRobotPose,robotPose.z);  // - odomData.pose.orientation.z;

    POINT_XYZ commandVelocity;
    
    /************************************************************************************************************************/
    commandVelocity.y = controlObject * 2.4 / PI;

    if (fabs(commandVelocity.y) > 0.5) commandVelocity.y = 0.5 * sign( commandVelocity.y );

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    // ROS_INFO("deadLine = %f, toLine = %f, target = %f.",deadLineDistance, trajectoryDistance, targetRobotPose);

    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.05)
    {
         commandVelocity.x = 0.05 * sign(commandVelocity.x);
    }
    

    if(deadLineDistance > 0.05)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }
}

bool backMotion(POINT_XYZ nextPoint,POINT_XYZ lastPoint)
{
    // angle control
    double pathDistance = deltaDistance(nextPoint , lastPoint); 

    Eigen::Vector2d pathVector( (nextPoint.x - lastPoint.x)/pathDistance, (nextPoint.y - lastPoint.y)/pathDistance );
    Eigen::Vector2d normalVector(pathVector(1), -pathVector(0)); // clockwise vector
    Eigen::Vector2d targetVector(nextPoint.x - robotPose.x, nextPoint.y - robotPose.y);

    double trajectoryDistance = distanceToAB(nextPoint , lastPoint, robotPose);  // left is positive; right is negative;
    double PathAngle = atan2( nextPoint.y - lastPoint.y, nextPoint.x - lastPoint.x);

    double aimLength = 0.5;
 
    double controlAngle = - 0.5 * PI * sign( trajectoryDistance );
    if( fabs( trajectoryDistance ) < aimLength)
    {
        controlAngle = - asin( trajectoryDistance / aimLength );
    }

    double targetRobotPose = constrainAngle(controlAngle + PathAngle);
    double  controlObject = deltaAngle(targetRobotPose,robotPose.z);  // - odomData.pose.orientation.z;

    POINT_XYZ commandVelocity;
    commandVelocity.y = 0;

    // if boundry mode, limit velocity
    double deadLineDistance = targetVector.transpose() * pathVector;
    // ROS_INFO("deadLine = %f, toLine = %f.",deadLineDistance, trajectoryDistance); 

    commandVelocity.x = 0.5 * deadLineDistance * cos(controlObject);

    // horizontal moving, limit velocity
    if( fabs(commandVelocity.x) > 0.3)
    {
         commandVelocity.x = 0.3 * sign(commandVelocity.x);
    }
    else if( fabs(commandVelocity.x) < 0.05)
    {
         commandVelocity.x = 0.05 * sign(commandVelocity.x);
    }

    if(deadLineDistance > 0.05)
    {
        targetStates.linear.x = commandVelocity.x;
        targetStates.angular.z = commandVelocity.y;
        return false;
    }
    else
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }
}

bool zeroTurning(double targetAngle)
{
    double deltaTheta = deltaAngle(targetAngle,robotPose.z);

    if(fabs(deltaTheta) > 5*PI/180)
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0.4 * sign(deltaTheta);
        return false;
    }
    else
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
        return true;
    }
}

POINT_XYZ normalizationVector(POINT_XYZ data)
{
    double length = sqrt(data.x*data.x + data.y*data.y + data.z*data.z);
    POINT_XYZ result;
    result.x = data.x/length;
    result.y = data.y/length;
    result.z = data.z/length;
    return result;
}

double stringToDouble(char *stringData,int num)
{
    bool expSymbol = false;

    bool pointValue = false;
    bool pointExpon = false;
    
    int signValue = 1;
    int signExpon = 1;
    
    double value = 0.0;
    double bitValue = 1.0;

    double expon = 0.0;
    double bitExpon = 1.0;

    for(int i = 0; i < num; i++)
    {
        if((stringData[i] == 'e')||(stringData[i] == 'E'))
        {
            expSymbol = true;
        }

        if( (stringData[i] >= '0') && (stringData[i] <= '9') )
        {
            if(expSymbol)
            {
                if(pointExpon)
                {
                    bitExpon = bitExpon * 0.1;
                    expon = expon + bitExpon * (stringData[i] - '0');
                }
                else
                {
                    expon = expon * 10 + (stringData[i] - '0');
                }
            }
            else
            {
                if(pointValue)
                {
                    bitValue = bitValue * 0.1;
                    value = value + bitValue * (stringData[i] - '0');
                }
                else
                {
                    value = value * 10 + (stringData[i] - '0');
                }
            }
        }

        if(stringData[i] == '-')
        {
            if(expSymbol)
            {
                signExpon = -1;
            }
            else
            {
                signValue = -1;
            }
        }

        if(stringData[i] == '.')
        {
            if(expSymbol)
            {
                pointExpon = true;
            }
            else
            {
                pointValue = true;
            }
        }
    }

    double result = signValue * value * pow(10,signExpon * expon);//

    return result;
}

void readBoundaryPoint()
{
    std::string boundary_path = "/home/rasp008/mower_ws/src/circuit_zigzag/data/boundary/";
    // std::string boundary_path = "/home/yat/catkin_ws/src/circuit_zigzag/data/boundary/";
    string file_path = boundary_path + "boundary.txt";

    std::ifstream boundaryStream(file_path.c_str(),ios::in);
    std::string line;

    //path data loading
    if(!boundaryStream.is_open()) 
    {
        ROS_ERROR("Can not open boundary.txt.");
    }
    
    vector<double> digitalData(12);

    Boundary_Point *sp = lastBoundary;
    Boundary_Feature *sf = lastFeature;

    while(!boundaryStream.eof() && getline(boundaryStream, line))
    {
        char *stringData,*startLine;
        startLine = &(line[0]);
        int dataNum = 0;

        while ((stringData = strtok(startLine," ")) != NULL && dataNum < 12)
        {
            if ( strspn(stringData, "0123456789.-eE") == strlen(stringData) )
            {
                digitalData[dataNum] =  atof(stringData);
                // digitalData[dataNum] = stringToDouble(stringData,strlen(stringData));
                dataNum++;
            }
            else
            {
                break;
            }

            // ROS_INFO("read initial point : NO.%d data is %f ,", dataNum-1, digitalData[dataNum-1]);

            startLine = NULL;
        }

        if(dataNum == 12)
        {
            switch ((int)(digitalData[0]))
            {
                case 0:
                    {
                        initialLast.num = digitalData[1];

                        if((int)(digitalData[2]) == 1)
                        {
                            missionSymbol.getYawOffset = true;
                        }

                        initialLast.x = digitalData[3];
                        initialLast.y = digitalData[4];
                        initialLast.theta = digitalData[8];
                    }
                    break;
                case 5:
                    {
                        Boundary_Point *newPointer = (struct Boundary_Point*)malloc(sizeof(struct Boundary_Point));
                        newPointer->next = NULL; 

                        sp->id = (int)(digitalData[1]);
                        sp->totalMile = digitalData[2];
                        sp->pose.x = digitalData[3];
                        sp->pose.y = digitalData[4];
                        sp->pose.theta = digitalData[5];

                        sp->next = newPointer;
                        sp = sp->next;
                    }
                    break;
                case 6:
                    {
                        Boundary_Feature *newPointer = (struct Boundary_Feature*)malloc(sizeof(struct Boundary_Feature));
                        newPointer->next = NULL; 

                        sf->id = (int)(digitalData[1]);
                        sf->point.x = digitalData[2];
                        sf->point.y = digitalData[3];
                        sf->theta.x = digitalData[4];
                        sf->theta.y = digitalData[5];
                        sf->inside = (int) digitalData[6];
                        sf->ratio = digitalData[7];
                        sf->covarXX = digitalData[8];
                        sf->covarXY = digitalData[9];
                        sf->covarYX = digitalData[10];
                        sf->covarYY = digitalData[11];

                        ROS_INFO("the NO.%d feature: startTheta = %5.15f, endTheta = %5.15f .",sf->id,sf->covarXX,sf->covarYY);

                        sf->next = newPointer;
                        sf = sf->next;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    if(missionSymbol.getYawOffset) ROS_INFO("get the boundary initiall data !");

    boundaryStream.close();
}

bool initialPosition()
{
    static int pointNum = 0;

    int initialNum = 50;

    static vector<POINT_XYZ> gyroData(initialNum);
    static vector<POINT_XYZ> positionData(initialNum);

    bool leftLowSpeed = (fabs(odomData.vel.x)<0.0001)?true:false;
    bool rightLowSpeed = (fabs(odomData.vel.y)<0.0001)?true:false;

    if(leftLowSpeed && rightLowSpeed && configData.enableGNSS)
    {
        gyroData[pointNum].x = imuData.gyro.x;
        gyroData[pointNum].y = imuData.gyro.y;
        gyroData[pointNum].z = imuData.gyro.z;

        positionData[pointNum].x = gnssData.pointECEF.x;
        positionData[pointNum].y = gnssData.pointECEF.y;
        positionData[pointNum].z = gnssData.pointECEF.z;

        pointNum++;
    }

    ROS_INFO("get the NO.%d GNSS point .",pointNum);

    if(pointNum>= initialNum)
    {

        imuData.bg = averagePoint(gyroData,pointNum);
        gnssData.baseECEF = averagePoint(positionData,pointNum);
        gnssData.baseBLH = ECEFtoBLH(gnssData.baseECEF);

        configData.gyroBias = imuData.bg;
        configData.baseBLH = gnssData.baseBLH;
        configData.baseECEF = gnssData.baseECEF;
        configData.enableBase = true;

        fout_point.setf(std::ios_base::showpoint);
        fout_point.precision(15);

        // line 1
        fout_point << gnssData.baseECEF.x << "    " << gnssData.baseECEF.y  << "   " <<  gnssData.baseECEF.z  << "   ";
        fout_point << gnssData.baseBLH.x << "   " << gnssData.baseBLH.y << "   " << gnssData.baseBLH.z << "   ";
        fout_point << imuData.bg.z << "    " << " 0 "  << "   " <<  " 1 " << std::endl;

        return true;
    }

    targetStates.linear.x = 0;
    targetStates.angular.z = 0;

    return false;
}

bool arriveBoundary()
{
    static int motionStep = 0;
    bool taskFinish = false;

    static POINT_XYZ fromPoint = robotPose;
    static POINT_XYZ backPoint = robotPose;
    static double targetAngle = constrainAngle(robotPose.z + PI);

    bool robotStop = ((fabs(odomData.vel.x) < 0.02)&&(fabs(odomData.vel.y) < 0.02))?true:false;

    if(motionStep == 0)
    {
        backPoint.x = robotPose.x - 0.8 * cos(robotPose.z);
        backPoint.y = robotPose.y - 0.8 * sin(robotPose.z);
        backPoint.z = robotPose.z;

        fromPoint = robotPose;

        motionStep = 1;

        ROS_INFO(" startPoint: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);
    }

    if(motionStep == 1)
    {
        if(backMotion(backPoint,fromPoint))
        {
            motionStep = 2;
        }
    }

    if(motionStep == 2)
    {
        if(robotStop)
        {
            motionStep = 3;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(motionStep == 3)
    {
        targetAngle = constrainAngle(robotPose.z + PI/2);
        if(zeroTurning(targetAngle))
        {
            // motionStep = 4;
        }

        if(detectData.rightSide && (!detectData.leftSide))
        {
            motionStep = 4;
        }
    }

    if(motionStep == 4)
    {
        if(robotStop)
        {
            motionStep = 0;

            taskFinish = true;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    ROS_INFO("arrive boundary -> motion step is : %d.",motionStep); 

    return taskFinish;
}

bool alongBoundary(bool &feutureFlag)
{
    static int motionStep = 0;
    bool taskFinish = false;

    static POINT_XYZ nextPoint = robotPose;
    static POINT_XYZ lastPoint = robotPose;

    static POINT_XYZ fromPoint = robotPose;
    static POINT_XYZ backPoint = robotPose;
    static int bunmNum = 0;

    static double deltaTheta = 0;
    static double lastTheta = 0;

    static double targetAngle = 0;

    bool robotStop = ((fabs(odomData.vel.x) < 0.02)&&(fabs(odomData.vel.y) < 0.02))?true:false;

    bool alongEdge = detectData.rightSide && (!detectData.leftSide); // along the doundary
    bool reverseEdge = (!detectData.rightSide) && detectData.leftSide; // reverse the doundary

    bool inRightSide = detectData.rightSide && detectData.leftSide;
    bool inLeftSide = (!detectData.rightSide) && (!detectData.leftSide);

    if(motionStep == 0)
    {
        motionStep = 1;
    }

    if(motionStep == 1)
    {
        if(inRightSide) // in the right side
        {
            double errorAngle = atan2(0.08132,0.34);

            nextPoint.x = robotPose.x + 0.18 * cos(robotPose.z + errorAngle);
            nextPoint.y = robotPose.y + 0.18 * sin(robotPose.z + errorAngle);
            nextPoint.z = robotPose.z;

            lastPoint = robotPose;
            
            motionStep = 2;
            // ROS_INFO("lift side -> right data : %f, left data: %f.",detectData.rightValue,detectData.leftValue); 
        }
        else if(inLeftSide) // in the left side
        {
            double errorAngle = atan2(0.08132,0.34);

            nextPoint.x = robotPose.x + 0.18 * cos(robotPose.z - errorAngle);
            nextPoint.y = robotPose.y + 0.18 * sin(robotPose.z - errorAngle);
            nextPoint.z = robotPose.z;

            lastPoint = robotPose;

            motionStep = 2;
            // ROS_INFO("right side -> right data : %f, left data: %f.",detectData.rightValue,detectData.leftValue);
        }
        else if(alongEdge) // in the boundary
        {
            double deltaL = 0.0832 * detectData.leftValue / (detectData.leftValue + detectData.rightValue);
            double deltaR = 0.0832 * detectData.rightValue / (detectData.leftValue + detectData.rightValue);

            double errorAngle = atan2(deltaR - deltaL,0.34);

            nextPoint.x = robotPose.x + 0.4 * cos(robotPose.z + errorAngle);
            nextPoint.y = robotPose.y + 0.4 * sin(robotPose.z + errorAngle);
            nextPoint.z = robotPose.z;

            // ROS_INFO("in side -> deltaR : %f, deltaL: %f.",deltaR,deltaL);

            lastPoint = robotPose;
        }
        
        if(reverseEdge)
        {
            motionStep = 3;
        }
        else
        {
            straightMotion(nextPoint,lastPoint);
        }        
    }

    if(motionStep == 2)
    {
        if(straightMotion(nextPoint,lastPoint))
        {
            motionStep = 3;
        }

        if(detectData.rightSide && (!detectData.leftSide)) // in the boundary
        {
            motionStep = 1;
        }
    }

    if(motionStep == 3)
    {
        if(robotStop)
        {
            motionStep = 4;

            deltaTheta = 0;
            lastTheta = robotPose.z;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(motionStep == 4)
    {
        double targetAngle = robotPose.z;
        if(detectData.rightSide && detectData.leftSide)
        {
            targetAngle = constrainAngle(robotPose.z + PI/2);
        }
        else if((!detectData.rightSide) && (!detectData.leftSide))
        {
            targetAngle = constrainAngle(robotPose.z - PI/2);
        }
        else if((!detectData.rightSide) && detectData.leftSide)
        {
            targetAngle = constrainAngle(robotPose.z + PI/2);
        }

        feutureFlag = true; 

        zeroTurning(targetAngle);

        deltaTheta += fabs(constrainAngle(robotPose.z - lastTheta));
        lastTheta = robotPose.z;

        // ROS_INFO("turning for boundary -> robot theta is : %f, delta theta is %f .",robotPose.z * 180 / PI,deltaTheta  * 180 / PI);

        if(deltaTheta > 2 * PI )
        {
            ROS_ERROR("Arround a circle, did not get the enable angle.");
            taskFinish = true;
        }

        if(detectData.rightSide && (!detectData.leftSide))
        {
            deltaTheta = 0;
            motionStep = 5;
        }
    }

    if(motionStep == 5)
    {
        if(robotStop)
        {
            motionStep = 1;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(detectData.rightBump || detectData.leftBump)
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;

        if(motionStep < 6) motionStep = 7;
    }

    if(motionStep == 6)
    {
        backPoint.x = robotPose.x - 0.2 * cos(robotPose.z);
        backPoint.y = robotPose.y - 0.2 * sin(robotPose.z);
        backPoint.z = robotPose.z;

        fromPoint = robotPose;

        motionStep = 7;

        ROS_INFO(" startPoint: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);
    }

    if(motionStep == 7)
    {
        if(backMotion(backPoint,fromPoint))
        {
            motionStep = 8;
        }
    }

    if(motionStep == 8)
    {
        bunmNum ++;
        if(bunmNum < 2)
        {
            motionStep = 4;
        }
        else
        {
            motionStep = 11;
            ROS_INFO("bumped and finish along boundary -> motion step is : %d.",motionStep); 
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(motionStep == 9)
    {
        if(robotStop)
        {
            motionStep = 10;
            targetAngle = constrainAngle(robotPose.z - PI/2);
            ROS_INFO(" turn to inside : angle = %f.",targetAngle);
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(motionStep == 10)
    {
        if(zeroTurning(targetAngle))
        {
            motionStep = 11;
        }
    }
    
    if(motionStep == 11)
    {
        if(robotStop)
        {
            motionStep = 0;

            taskFinish = true;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    // ROS_INFO("along boundary -> motion step is : %d.",motionStep); 

    return taskFinish;
}

bool randomMotion()
{
    static int motionStep = 0;
    bool taskFinish = false;

    static POINT_XYZ nextPoint = robotPose;
    static POINT_XYZ lastPoint = robotPose;

    static POINT_XYZ fromPoint = robotPose;
    static POINT_XYZ backPoint = robotPose;

    static double targetAngle = 0;

    bool robotStop = ((fabs(odomData.vel.x) < 0.002)&&(fabs(odomData.vel.y) < 0.002))?true:false;

    if(motionStep == 0)
    {
        nextPoint.x = robotPose.x + 50 * cos(robotPose.z);
        nextPoint.y = robotPose.y + 50 * sin(robotPose.z);
        
        lastPoint = robotPose;

        motionStep = 1;
    }

    if(motionStep == 1)
    {
        double moveDistance = sqrt(pow(robotPose.x - lastPoint.x,2) + pow(robotPose.y - lastPoint.y,2));

        bool enableBack = (moveDistance > 0.3)?true:false;

        if(straightMotion(nextPoint,lastPoint))
        {
            motionStep = 0;
        }

        if((!detectData.rightSide) || (!detectData.leftSide))
        {
            if(enableBack) 
            {
                motionStep = 2;
            }
            else
            {
                motionStep = 4;
            }
        }

        if(detectData.rightBump || detectData.leftBump)
        {
            if(enableBack) 
            {
                motionStep = 2;
            }
            else
            {
                motionStep = 4;
            }
        }
    }

    if(motionStep == 2)
    {
        if(robotStop)
        {
            backPoint.x = robotPose.x - 0.2 * cos(robotPose.z);
            backPoint.y = robotPose.y - 0.2 * sin(robotPose.z);
            backPoint.z = robotPose.z;

            fromPoint = robotPose;

            motionStep = 3;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(motionStep == 3)
    {
        if(backMotion(backPoint,fromPoint))
        {
            motionStep = 4;
        }
    }

    if(motionStep == 4)
    {        
        if(robotStop)
        {
            targetAngle = constrainAngle(robotPose.z + (rand()%360 + 1.0) * PI / 180);
            motionStep = 5;
            ROS_INFO("random motion : robot theta is %f, random theta is  %f.",robotPose.z * RadToDegree, targetAngle * RadToDegree); 
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    } 

    if(motionStep == 5)
    {
        if(zeroTurning(targetAngle))
        {
            motionStep = 6;
        }
    }

    if(motionStep == 6)
    {
        if(robotStop)
        {
            motionStep = 0;
        }

        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    if(solutionData.initialState)
    {

        motionStep = 0;
        taskFinish = true;
    }

    ROS_INFO("random motion -> motion step is : %d.",motionStep);

    return taskFinish;
}

void initialPublish()
{
    geometry_msgs::PoseStamped initialPose;
    initialPose.header.stamp = ros::Time::now();
    initialPose.header.frame_id = "InitialState";

    initialPose.pose.position.x = configData.baseECEF.x;
    initialPose.pose.position.y = configData.baseECEF.y;
    initialPose.pose.position.z = configData.baseECEF.z;
    
    initialPose.pose.orientation.x = solutionData.robotDR.x;  // robot ENU position X axis
    initialPose.pose.orientation.y = solutionData.robotDR.y;  // robot ENU position Y axis
    initialPose.pose.orientation.z = solutionData.robotEuler.z;  // robot euler in Z axis
    initialPose.pose.orientation.w = solutionData.gyroBias.z;  // robot angle rate in Z axis

    initial_pub.publish(initialPose);
}

bool initialBoundary()
{
    static int motionStep = 0;
    bool taskFinish = false;

    if(motionStep == 0)
    {
        motionStep = 1;

        ROS_INFO(" startPoint: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);
    }

    if(motionStep == 1)
    {
        if(initialPosition())
        {
            motionStep = 2;
        }
    }

    if(motionStep == 2)
    {
        if(arriveBoundary())
        {
            motionStep = 3;
        }
        recordBoundaryData(false);
    }

    if(motionStep == 3)
    {
        bool feutureFlag = false;
        if(alongBoundary(feutureFlag))
        {
            missionSymbol.alongFinish = true;
            motionStep = 4;
        }
        recordBoundaryData(feutureFlag);
    }

    if(motionStep == 4)
    {
        if(randomMotion())
        {
            initialPublish();
            motionStep = 5;
        }
    }

    if(motionStep == 5)
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;

        taskFinish = true;
        motionStep = 0;
    }

    if((motionStep >= 3) &&(!detectData.signalSide))
    {
        targetStates.linear.x = 0;
        targetStates.angular.z = 0;
    }

    vel_pub.publish(targetStates);

    // ROS_INFO(" robot pose: x = %f , y = %f , theta = %f.",robotPose.x,robotPose.y,robotPose.z);

    return taskFinish;
}

void attitudeFilterSolution(CONFIG_PARAMETER &configMessage,SOLUTION_PARAMETER &solutionMessage)
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

    // update priori estimate yaw states
    solutionMessage.robotEuler.x = configMessage.euler.x;
    solutionMessage.robotEuler.y = configMessage.euler.y;
    solutionMessage.robotEuler.z = solutionMessage.robotEuler.z + dt * (configMessage.gyro.z - solutionMessage.gyroBias.z); //deltaYaw; // dt * configMessage.gyro.z; //

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,2,2> P0;
    P0.setZero(2,2);
    P0(0,0) = pow(DegreeToRad*0.05,2); // initial gyro angle error
    P0(1,1) = pow(DegreeToRad/64/50,2); // initial gyro angle rate error

    static Eigen::Matrix<double,2,2> P_p = P0;

    // configure system state Noise covariance Q
    Eigen::Matrix<double,2,2> Q;
    Q.setZero(2,2);
    Q(0,0) = pow(DegreeToRad * 0.05,2); // gyro angle random noise
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
    bool disableGPS = (sqrtValue(configMessage.gnssVel)<0.15)?true:false;
    bool disableOdom = (fabs(configData.odomVel.z)<0.15)?true:false;

    // posteriori evaluation
    double yawGPS = constrainAngle(atan2(configMessage.gnssVel.y,configMessage.gnssVel.x) - 0.5 * PI);
    if(configMessage.odomVel.z < 0)
    {
        yawGPS = constrainAngle(yawGPS + PI);
    }

    double deltaTheta = deltaAngle(yawGPS,solutionMessage.robotEuler.z); 
    if(disableGPS || disableOdom || (!configData.enableGNSS) || fabs(deltaTheta) > 60 * DegreeToRad)
    {
        yawGPS = solutionMessage.robotEuler.z;
    }
    else
    {
        // update Prior estimate covariance
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

    double currentMileage = configMessage.odomMil.z;  // current mileage measured by odom
    static double mileage_prev = currentMileage; 
    double deltaMileage = currentMileage - mileage_prev;
    mileage_prev = currentMileage;

    Eigen::Vector2d positionBody(0,deltaMileage);
    Eigen::Vector2d velocityBody(0,configMessage.odomVel.z);

    Eigen::Vector2d positionNav = Cbn * positionBody;
    Eigen::Vector2d velocityNav = Cbn * velocityBody;

    solutionData.robotEKF.x = solutionData.robotEKF.x + positionNav(0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + positionNav(1);
    solutionData.robotVel.x = velocityNav(0);
    solutionData.robotVel.y = velocityNav(1);

    // configure initial Prior estimate covariance
    Eigen::Matrix<double,4,4> P0;
    P0.setZero(4,4);
    P0(0,0) = pow( 0.05 , 2 ); // initial X axis position noise
    P0(1,1) = pow( 0.05 , 2 ); // initial Y axis position noise
    P0(2,2) = pow( 0.1 , 2 ); // initial X axis velocity noise
    P0(3,3) = pow( 0.1 , 2 ); // initial Y axis velocity noise

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
    R(0,0) = pow( 0.05 , 2 );  // X axis position noise of GNSS
    R(1,1) = pow( 0.05 , 2 );  // Y axis position noise of GNSS
    R(2,2) = pow( 0.01 , 2 );  // X axis velocity noise of GNSS
    R(3,3) = pow( 0.01 , 2 );  // Y axis velocity noise of GNSS

    Eigen::Matrix<double,4,4> K;
    K.setZero(4,4);

    static POINT_XYZ lastENU = configMessage.gnssENU;

    double deltaPointENU = sqrt(pow(configMessage.gnssENU.x - lastENU.x,2) + pow(configMessage.gnssENU.y - lastENU.y,2)) - deltaMileage;
    bool enableEKF = (fabs(deltaPointENU) < 2)?true:false;

    if(configMessage.enableGNSS && enableEKF )
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
    else
    {
        if(!enableEKF) ROS_ERROR("the gnss position is Outliers.");
    }

    Eigen::Matrix<double,4,1> Y;
    Y.setZero(4,1);
    Y(0,0) = configMessage.gnssENU.x - lastENU.x - positionNav(0);
    Y(1,0) = configMessage.gnssENU.y - lastENU.y - positionNav(1);
    Y(2,0) = 0; // sqrtValue(configMessage.gnssVel) * cos(solutionMessage.robotEuler.z + PI/2) - solutionData.robotVel.x
    Y(3,0) = 0; // sqrtValue(configMessage.gnssVel) * sin(solutionMessage.robotEuler.z + PI/2) - solutionData.robotVel.y

    // update posteriori evaluation states
    Eigen::Matrix<double,4,1> X_f = K * Y;

    // update posteriori estimate covariance
    Eigen::Matrix<double,4,4> P_f = (I - K*H)*P_p;
    P_p = P_f;

    solutionData.robotEKF.x = solutionData.robotEKF.x + X_f(0,0);
    solutionData.robotEKF.y = solutionData.robotEKF.y + X_f(1,0);
    solutionData.robotVel.x = solutionData.robotVel.x + X_f(2,0);
    solutionData.robotVel.y = solutionData.robotVel.y + X_f(3,0);

    lastENU = configMessage.gnssENU;
}

void deadReckoningSolution(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    // double currentYaw = configMessage.euler.z;
    double currentYaw = solutionMessage.robotEuler.z;
    double currentMileage = 0.5*(configMessage.odomMil.x + configMessage.odomMil.y);  // current mileage measured by odom

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

void initialParameter(CONFIG_PARAMETER &configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    POINT_XYZ zeroPoint;
    zeroPoint.x = 0;
    zeroPoint.y = 0;
    zeroPoint.z = 0;
    
    missionSymbol.getYawOffset = false;
    missionSymbol.motionFinish = false;
    missionSymbol.initialFinish = false;
    missionSymbol.alongFinish = false;
    missionSymbol.recordBoundary = false;

    configMessage.baseENU = zeroPoint; // origin point , coordinates is (0,0,0)

    configMessage.enableBase = false;
    solutionMessage.initialState = false;

    readBoundaryPoint();

    if(missionSymbol.getYawOffset)
    {
        mowerPose.x = 0;
        mowerPose.y = 0;
        mowerPose.theta = constrainAngle(0.5 * PI + initialLast.theta);

        double rotateAngle = initialLast.theta;

        solutionMessage.robotEuler.x = configMessage.euler.x;
        solutionMessage.robotEuler.y = configMessage.euler.y;
        solutionMessage.robotEuler.z = rotateAngle;

        // solutionMessage.robotEKF = getCoordinatesENU(configMessage.gnssECEF,configMessage.baseECEF,configMessage.baseBLH);
        solutionMessage.robotEKF.x = zeroPoint.x;
        solutionMessage.robotEKF.y = zeroPoint.y;
        solutionMessage.robotEKF.z = zeroPoint.z;

        solutionMessage.robotVel.x = cos(rotateAngle) * configMessage.odomVel.z;
        solutionMessage.robotVel.y = sin(rotateAngle) * configMessage.odomVel.z;
        solutionMessage.robotVel.z = 0;

        solutionMessage.robotDR = solutionMessage.robotEKF;

        solutionMessage.robotPose.x = solutionMessage.robotEuler.x;
        solutionMessage.robotPose.y = solutionMessage.robotEuler.y;
        solutionMessage.robotPose.z = constrainAngle(0.5 * PI + rotateAngle);

        ROS_INFO("get the yaw offset is : %f", rotateAngle * RadToDegree);
    }
    else
    {
        mowerPose.x = zeroPoint.x;
        mowerPose.y = zeroPoint.y;
        mowerPose.theta = 0.5 * PI;

        initialLast.theta = 0;
    }
}

double getInitialBias(double filterAngle,int num,vector<POINT_XYZ> YawData)
{
    vector<POINT_XY> deltaBias(filterYawNum);
    double averageYawBias = 0;
    for(int i = 0; i < num; i++)
    {
        deltaBias[i].x = constrainAngle(YawData[i].x - YawData[i].y);
        deltaBias[i].y = constrainAngle(deltaBias[i].x - filterAngle);
        averageYawBias += deltaBias[i].y / num;
    }

    POINT_XY biasMedian = medianVaule(deltaBias,num);

    double biasYawA = constrainAngle(biasMedian.x);
    double biasYawB = constrainAngle(biasMedian.y + filterAngle);

    ROS_INFO("estimation initial NO.%d angle -> delta yaw A is = %f,delta yaw B is = %f", num, biasYawA * RadToDegree, biasYawB * RadToDegree);

    return biasYawB;
}


void combinationNavigation(CONFIG_PARAMETER configMessage,SOLUTION_PARAMETER &solutionMessage)
{
    static int YawDataNum = 0;
    static vector<POINT_XYZ> YawData(filterYawNum);
    static int boundaryNum = 0;

    static double biasYaw = 0;
    static bool getStartBais = false;
    
    // bool robotStuck = ((sqrtValue(configMessage.gnssVel) < 0.05)&&(fabs(configMessage.odomVel.z) > 0.2))?true:false;
    bool robotStuck = false;

    if(configData.enableBase)
    {
        if(missionSymbol.getYawOffset)
        {
            getStartBais = true;
        }

        if(getStartBais)
        {
            if(robotStuck)
            {
                ROS_ERROR("the robot is stuck.");
            }
            else
            {
                // update attitude Kalman solution
                attitudeFilterSolution(configMessage,solutionMessage);

                // update the position and velocity Kalman filter solution
                positionFilterSolution(configMessage,solutionMessage);

                // update the dead reckoning solution
                deadReckoningSolution(configMessage,solutionMessage);
            }

            mowerPose.x = solutionMessage.robotDR.x;
            mowerPose.y = solutionMessage.robotDR.y;
            mowerPose.theta = solutionMessage.robotPose.z;
        }
        else
        {
            mowerPose = getRealTimePosition();
        }

        bool enableVelocity = (sqrtValue(configMessage.gnssVel) > 0.15)?true:false;
        bool enableOdom = (fabs(configMessage.odomVel.z) > 0.15)?true:false;

        if(enableVelocity && enableOdom && configData.enableGNSS)
        {
            double gnssTheta = atan2(configMessage.gnssVel.y,configMessage.gnssVel.x);
            if(configMessage.odomVel.z < 0)
            {
                gnssTheta = constrainAngle(gnssTheta + PI);
            }

            double deltaTheta = deltaAngle(gnssTheta,mowerPose.theta);
            double deltaBias = constrainAngle(deltaTheta - biasYaw);

            if(YawDataNum == 0)
            {
                biasYaw = deltaTheta;
            }
            else
            {
                biasYaw = constrainAngle( biasYaw + 0.005 * deltaBias);
            }

            if(YawDataNum < filterYawNum)
            {
                YawData[YawDataNum].x = gnssTheta;  // yaw from gnss
                YawData[YawDataNum].y = mowerPose.theta;   // yaw from imu
                YawData[YawDataNum].z = biasYaw;

                YawDataNum ++;
            }
        }

        if((!getStartBais) && (YawDataNum >= 100))
        {
            double rotateAngle = getInitialBias(biasYaw,YawDataNum,YawData);

            initialLast.theta = rotateAngle;

            Boundary_Point *sp = rawBoundaryPoint; // struct pointer

            while(sp->next != NULL)
            {
                YAT_POSE lastPose = sp->pose;
                sp->pose.x = lastPose.x * cos(rotateAngle) - lastPose.y * sin(rotateAngle);
                sp->pose.y = lastPose.x * sin(rotateAngle) + lastPose.y * cos(rotateAngle);
                sp->pose.theta = constrainAngle(lastPose.theta + rotateAngle);
                sp = sp->next;
            }

            YAT_POSE lastPose = mowerPose;
            mowerPose.x = lastPose.x * cos(rotateAngle) - lastPose.y * sin(rotateAngle);
            mowerPose.y = lastPose.x * sin(rotateAngle) + lastPose.y * cos(rotateAngle);
            mowerPose.theta = constrainAngle(lastPose.theta + rotateAngle);

            ROS_INFO("estimation first time -> yaw bias is = %f", rotateAngle * RadToDegree);

            solutionMessage.robotEuler.x = configMessage.euler.x;
            solutionMessage.robotEuler.y = configMessage.euler.y;
            solutionMessage.robotEuler.z = constrainAngle(mowerPose.theta - 0.5 * PI);

            solutionMessage.robotEKF.x = mowerPose.x;
            solutionMessage.robotEKF.y = mowerPose.y;
            solutionMessage.robotEKF.z = 0;

            solutionMessage.robotVel.x = cos(rotateAngle) * configMessage.odomVel.z;
            solutionMessage.robotVel.y = sin(rotateAngle) * configMessage.odomVel.z;
            solutionMessage.robotVel.z = 0;

            solutionMessage.robotDR = solutionMessage.robotEKF;

            solutionMessage.robotPose.x = solutionMessage.robotEuler.x;
            solutionMessage.robotPose.y = solutionMessage.robotEuler.y;
            solutionMessage.robotPose.z = mowerPose.theta;

            for(int i = 0 ; i < YawDataNum ; i++)
            {
                YawData[i].y = constrainAngle(YawData[i].y + rotateAngle);
            }

            getStartBais = true;
        }

        // average filter
        if(missionSymbol.alongFinish)
        {
            biasYaw = getInitialBias(biasYaw,YawDataNum,YawData);

            initialData.theta = biasYaw;

            ROS_INFO("estimation finial -> delta yaw is = %f", biasYaw * RadToDegree);
            ROS_INFO("finish intialization the Yaw.");

            solutionMessage.robotEuler.x = configMessage.euler.x;
            solutionMessage.robotEuler.y = configMessage.euler.y;
            solutionMessage.robotEuler.z = constrainAngle(mowerPose.theta + biasYaw - 0.5 * PI);

            // solutionMessage.robotEKF = getCoordinatesENU(configMessage.gnssECEF,configMessage.baseECEF,configMessage.baseBLH);
            solutionMessage.robotEKF.x = 1.0 * cos(mowerPose.theta + biasYaw + 0.5 * PI);
            solutionMessage.robotEKF.y = 1.0 * sin(mowerPose.theta + biasYaw + 0.5 * PI);
            solutionMessage.robotEKF.z = 0;

            fout_yawdata.setf(std::ios_base::showpoint);
            fout_yawdata.precision(15);

            for(int i = 0; i < YawDataNum; i++)
            {
                fout_yawdata << i << " " << YawData[i].x << " " << YawData[i].y  << " " << YawData[i].z  << " " << " 0 " << std::endl;
            }
            
            fout_yawdata.close();

            solutionMessage.initialState = true;
        }

        boundaryNum++;

        ROS_INFO("the NO.%d edge point is: x = %f, y = %f",boundaryNum,solutionMessage.robotEKF.x,solutionMessage.robotEKF.y);

    }
}

void imuCallback(const geometry_msgs::PoseArrayConstPtr& msg_imu)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    imuData.update = true;

    imuData.acc.x = msg_imu->poses[1].position.x; // acceleration data
    imuData.acc.y = msg_imu->poses[1].position.y;
    imuData.acc.z = msg_imu->poses[1].position.z;

    imuData.gyro.x = msg_imu->poses[1].orientation.x * DegreeToRad; // gyro data
    imuData.gyro.y = msg_imu->poses[1].orientation.y * DegreeToRad;
    imuData.gyro.z = msg_imu->poses[1].orientation.z * DegreeToRad;

    imuData.euler.x =  msg_imu->poses[0].position.x * DegreeToRad; //euler angle
    imuData.euler.y =  msg_imu->poses[0].position.y * DegreeToRad;
    imuData.euler.z =  msg_imu->poses[0].position.z * DegreeToRad;

    configData.updateIMU = true;

    configData.accel.x = earth_g * msg_imu->poses[1].position.x; // acceleration data
    configData.accel.y = earth_g * msg_imu->poses[1].position.y;
    configData.accel.z = earth_g * msg_imu->poses[1].position.z;

    configData.gyro.x = msg_imu->poses[1].orientation.x * DegreeToRad; // gyro data
    configData.gyro.y = msg_imu->poses[1].orientation.y * DegreeToRad;
    configData.gyro.z = msg_imu->poses[1].orientation.z * DegreeToRad;

    configData.euler.x =  msg_imu->poses[0].position.x * DegreeToRad; //euler angle
    configData.euler.y =  msg_imu->poses[0].position.y * DegreeToRad;
    configData.euler.z =  msg_imu->poses[0].position.z * DegreeToRad;

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 2
    fout_point << msg_imu->poses[0].position.x << "    " << msg_imu->poses[0].position.y  << "   " <<  msg_imu->poses[0].position.z  << "   ";
    fout_point << msg_imu->poses[0].orientation.x << "   " << msg_imu->poses[0].orientation.y << "   " << msg_imu->poses[0].orientation.z << "   ";
    fout_point << msg_imu->poses[0].orientation.w << "    " << " 0 "  << "   " <<  " 6 " << std::endl;

    // line 3
    fout_point << msg_imu->poses[1].position.x << "    " << msg_imu->poses[1].position.y  << "   " <<  msg_imu->poses[1].position.z  << "   ";
    fout_point << msg_imu->poses[1].orientation.x << "   " << msg_imu->poses[1].orientation.y << "   " << msg_imu->poses[1].orientation.z << "   ";
    fout_point << msg_imu->poses[1].orientation.w << "    " << dt  << "   " <<  " 7 " << std::endl;
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

    static int leftTime = 0;
    static int rightTime = 0;

    bool bumpMsgLeft = (((int) msg_alf.poses[0].orientation.x) == 1)?true:false;
    bool bumpMsgRight = (((int) msg_alf.poses[0].orientation.y) == 1)?true:false;
    
    leftTime = bumpMsgLeft ? (leftTime + 1) : 0;
    detectData.leftBump = (leftTime >= 2) ? true : false;

    rightTime = bumpMsgRight ? (rightTime + 1) : 0;
    detectData.rightBump = (rightTime >= 2) ? true : false;

    detectData.leftUp = (((int) msg_alf.poses[0].orientation.z) == 1)?true:false;
    detectData.rightUp = (((int) msg_alf.poses[0].orientation.w) == 1)?true:false;

    odomData.update = true;

    odomData.mil.x = msg_alf.poses[1].orientation.x; // left wheel mileage
    odomData.mil.y = msg_alf.poses[1].orientation.y; // right wheel mileage
    odomData.mil.z = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y); // mileage

    odomData.vel.x = msg_alf.poses[1].orientation.z; // left wheel velocity
    odomData.vel.y = msg_alf.poses[1].orientation.w; // right wheel velocity
    odomData.vel.z = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w); // velocity

    configData.odomMil.x = msg_alf.poses[1].orientation.x; // left wheel mileage
    configData.odomMil.y = msg_alf.poses[1].orientation.y; // right wheel mileage
    configData.odomMil.z = 0.5*(msg_alf.poses[1].orientation.x + msg_alf.poses[1].orientation.y); // mileage

    configData.odomVel.x = msg_alf.poses[1].orientation.z; // left wheel velocity
    configData.odomVel.y = msg_alf.poses[1].orientation.w; // right wheel velocity
    configData.odomVel.z = 0.5*(msg_alf.poses[1].orientation.z + msg_alf.poses[1].orientation.w); // velocity

    if(configData.updateIMU) locationDR();

    fout_point.setf(std::ios_base::showpoint);
    fout_point.precision(15);

    // line 2
    fout_point << msg_alf.poses[0].position.x << "    " << msg_alf.poses[0].position.y  << "   " <<  msg_alf.poses[0].position.z  << "   ";
    fout_point << msg_alf.poses[0].orientation.x << "   " << msg_alf.poses[0].orientation.y << "   " << msg_alf.poses[0].orientation.z << "   ";
    fout_point << msg_alf.poses[0].orientation.w << "    " << " 0 "  << "   " <<  " 2 " << std::endl;

    // line 3
    fout_point << msg_alf.poses[1].position.x << "    " << msg_alf.poses[1].position.y  << "   " <<  msg_alf.poses[1].position.z  << "   ";
    fout_point << msg_alf.poses[1].orientation.x << "   " << msg_alf.poses[1].orientation.y << "   " << msg_alf.poses[1].orientation.z << "   ";
    fout_point << msg_alf.poses[1].orientation.w << "    " << " 0 "  << "   " <<  " 3 " << std::endl;
}

void gnssMessageCallback(const geometry_msgs::PoseStampedConstPtr& msg_position,
                         const geometry_msgs::PoseStampedConstPtr& msg_state)
{
    static ros::Time lastTime = ros::Time::now();
    double dt = ros::Time::now().toSec() - lastTime.toSec();
    lastTime = ros::Time::now();

    gnssData.pointBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    gnssData.pointBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    gnssData.pointBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;

    gnssData.pointECEF = BLHtoECEF(gnssData.pointBLH);

    gnssData.direction = constrainAngle( PI/2 - msg_state->pose.orientation.z * DegreeToRad);
    gnssData.velocity = msg_state->pose.orientation.w * 0.514444;

    configData.updateGNSS = true;
    configData.enableGNSS = ((int)(msg_state->pose.orientation.y) == 'A')?true:false;  // true is available, false is invalid

    configData.gnssBLH.x = DegreeToRad * msg_position->pose.orientation.x;
    configData.gnssBLH.y = DegreeToRad * msg_position->pose.orientation.y;
    configData.gnssBLH.z = msg_position->pose.orientation.z + msg_position->pose.orientation.w;
    configData.gnssECEF = BLHtoECEF(configData.gnssBLH);
    if( configData.enableBase ) configData.gnssENU = getCoordinatesENU(configData.gnssECEF,configData.baseECEF,configData.baseBLH);
                           else configData.gnssENU = configData.baseENU;

    double speedGround = 0.5144444 * msg_state->pose.orientation.w;
    double directionGround = constrainAngle(PI/2 - msg_state->pose.orientation.z * DegreeToRad);

    configData.gnssVel.x = speedGround * cos(directionGround);
    configData.gnssVel.y = speedGround * sin(directionGround);
    configData.gnssVel.z = 0;

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
}

nav_msgs::OccupancyGrid postOccupancyMap(MAP_GRID *gridMap)
{
    // gridMapType parametre = getGridParametre(gridMap);

    // ROS_INFO("Creating map rviz");
    nav_msgs::OccupancyGrid ocmap;
    ocmap.header.frame_id = "initialMap";
    ocmap.header.stamp = ros::Time::now();

    ocmap.info.height = gridMap->rangeY;
    ocmap.info.width = gridMap->rangeX;
    ocmap.info.resolution = 1.0 / gridMap->scale;
    ocmap.info.origin.position.x = gridMap->offsetX;
    ocmap.info.origin.position.y = gridMap->offsetY;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_boundary");
    ROS_INFO_STREAM("initial the boundary ...");

    ros::NodeHandle nh;

    //Publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    initial_pub = nh.advertise<geometry_msgs::PoseStamped>("/initial_pose", 1);
    boundary_pub = nh.advertise<geometry_msgs::PoseStamped>("/boundary_pose", 1);
    ros::Publisher occmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/initialMap", 1, true);

    //Subcriber
    ros::Subscriber odom_sub  = nh.subscribe("/alf001_dis", 1,odomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 1, imuCallback);

    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkPosition_sub(nh,"gnss_position",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> rtkState_sub(nh,"gnss_state",1);
    message_filters::Synchronizer<sync_policy_classifiction> sync(sync_policy_classifiction(10), rtkPosition_sub,rtkState_sub);
    sync.registerCallback(boost::bind(&gnssMessageCallback, _1, _2));

    srand(time(NULL));

    rawBoundaryPoint->next = NULL;
    rawFeaturePoint->next = NULL;

    correctBoundary->next = NULL;
    correctFeature->next = NULL;

    filterBoundary->next = NULL;
    filterFeature->next = NULL;

    lastBoundary->next = NULL;
    lastFeature->next = NULL;

    MAP_GRID *gridMap = (MAP_GRID*)malloc(sizeof(MAP_GRID));
    gridMap->data = NULL;

    initialParameter(configData,solutionData);

    ros::Rate loop_rate(10);
    ros::Time prev_whileloop = ros::Time::now();
    while(ros::ok())
    {
        if(missionSymbol.motionFinish)
        {
            nav_msgs::OccupancyGrid postMap = postOccupancyMap(gridMap);
            occmap_pub.publish(postMap);
        }
        else
        {
            combinationNavigation(configData,solutionData);

            if(imuData.update && detectData.update)
            {
                if(initialBoundary())
                {
                    getBoundaryPoint(gridMap);

                    missionSymbol.motionFinish = true;
                    ROS_INFO("initialization is finished.");
                }

                if(!detectData.signalSide) ROS_INFO("initialization no signal data");
            }
            else
            {
                if(!gnssData.update) ROS_INFO("initialization no GNSS data");
                if(!imuData.update) ROS_INFO("initialization no Odom data");
                if(!imuData.update) ROS_INFO("initialization no IMU data");
                
                targetStates.linear.x = 0;
                targetStates.angular.z = 0;
                vel_pub.publish(targetStates);
            }
        }

        imuData.update = false;
        odomData.update = false;
        detectData.update = false;
        gnssData.update = false;

        configData.enableGNSS = false;

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
