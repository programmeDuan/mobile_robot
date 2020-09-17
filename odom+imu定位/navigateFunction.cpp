#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <stack>

#include "core.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
#define PI 3.1415926535897932384626433
#define DegreeToRad 3.1415926535897932384626433/180
#define RadToDegree 180/3.1415926535897932384626433

#define earth_A 6378137.0 //metre
#define earth_W 0.000072921151467 // rad/s
#define earth_F 1/298.257223563 //
#define earth_E2 earth_F*(2-earth_F) //

#define earth_g 9.7803267715 // gravity acceleration in equator [m/s^2]
#define Beta0 0.00531360853 
#define Beta1 0.00000585905
#define Beta2 -0.00000307246

typedef struct
{
	double  x; // coordinate in X axis
	double  y; // coordinate in Y axis
}POINT_XY;

typedef struct
{
	double  x; // coordinate in X axis
	double  y; // coordinate in Y axis
	double  z; // coordinate in Z axis
}POINT_XYZ;

typedef struct
{
    bool update;

	bool leftSide; // left is in or out : in is true, out is false
    bool rightSide; // right is in or out : in is true, out is false
    bool signalSide;

    double leftValue;
    double rightValue;

	bool leftBump;
    bool rightBump; 
    bool leftUp; 
    bool rightUp; 
}DETECT_TYPE;

typedef struct
{
	double q[4];
}Quaternion_Type;

typedef struct configType
{
    bool enableBase; // true is available, false is invalid
    POINT_XYZ baseBLH;
    POINT_XYZ baseENU; // origin point , coordinates is (0,0,0)
    POINT_XYZ baseECEF;

    bool enableGNSS;  // true is available, false is invalid
    bool updateGNSS;
    double dtGNSS; // delta time from last RTK data update to now [Hz]
    POINT_XYZ gnssBLH;
    POINT_XYZ gnssENU;
    POINT_XYZ gnssECEF;

    POINT_XYZ gnssVel;

    bool updateIMU;
    double dtIMU;
    double imuTemperature; 
    POINT_XYZ euler; // sensor Angle
    POINT_XYZ accel;
    POINT_XYZ gyro;
    POINT_XYZ gyroBias;

    double dtOdom;
    POINT_XYZ odomMil;
    POINT_XYZ odomVel;
}CONFIG_PARAMETER;

typedef struct solutionType
{
    bool initialState;

    POINT_XYZ gyroBias;

    // POINT_XYZ robotBLH;   // is equal to robotPosition
    // POINT_XYZ gnssBLH;
    // POINT_XYZ gnssENU;
    // POINT_XYZ gnssECEF;

    double Qbn[4];
    POINT_XYZ robotEuler; // roll, pitch and yaw
    POINT_XYZ robotPose; // roll and pitch is equal to euler angle, yaw is euler.z + PI/2
    POINT_XYZ robotVel;
    POINT_XYZ robotDR;
    POINT_XYZ robotEKF;
    
    POINT_XYZ robotENU;
    POINT_XYZ robotBLH;
    POINT_XYZ robotECEF;
}SOLUTION_PARAMETER;

// deal with the margin -180 <=> 180
double constrainAngle(double angle)
{
    double trueAngle = angle;
    while (trueAngle > PI)
    {
        trueAngle -= 2*PI;
    }

    while (trueAngle < -PI)
    {
        trueAngle += 2*PI;
    }

    return trueAngle;
}

// deal with the margin -180 <=> 180
double deltaAngle(double targetAngle, double currentAngle)
{
    return constrainAngle(targetAngle - currentAngle);
}

double sqrtValue(POINT_XYZ data)
{
    return sqrt(data.x * data.x + data.y * data.y + data.z * data.z);
}

Eigen::VectorXd zeroVector(int row)
{
    Eigen::VectorXd result;
    for(int i = 0; i < row; i++)
    {
        result(i) = 0;
    } 
    return result;
}

Eigen::MatrixXd zeroMatrix(int row,int column)
{
    Eigen::MatrixXd result(row,column);
    for(int i = 0; i < row; i++)
    {
        for(int j = 0; j < column; j++)
        {
            result(i,j) = 0;
        }
    } 
    return result;
}

Eigen::MatrixXd unitMatrix(int num)
{
    Eigen::MatrixXd result(num,num);
    for(int i = 0; i < num; i++)
    {
        for(int j = 0; j < num; j++)
        {
            result(i,j) = (i == j)?1:0;
        }
    } 
    return result;
}

POINT_XYZ eigenToXYZ(Eigen::Vector3d data)
{
    POINT_XYZ point;
    point.x = data(0);    
    point.y = data(1);    
    point.z = data(2);
    return point;
}

POINT_XYZ averagePoint(vector<POINT_XYZ> pointData,int num)
{
    POINT_XYZ average;
    average.x = 0;
    average.y = 0;
    average.z = 0;
    for(int i = 0 ; i < num; i++)
    {
        average.x += pointData[i].x/num;
        average.y += pointData[i].y/num;
        average.z += pointData[i].z/num;
    }
    return average;
}

POINT_XYZ BLHtoECEF(POINT_XYZ PointBLH)
{
    POINT_XYZ coordinatesECEF;
    double h = PointBLH.z;

    double phi = PointBLH.x;
    double lambda = PointBLH.y;

    double a = earth_A; //metre
    double f = earth_F;
    double e2 = f*(2-f);
    double N = a/sqrt(1-e2*sin(phi)*sin(phi));

    coordinatesECEF.x = (N+h)*cos(phi)*cos(lambda);
    coordinatesECEF.y = (N+h)*cos(phi)*sin(lambda);
    coordinatesECEF.z = (N*(1-e2)+h)*sin(phi);

    return coordinatesECEF;
}

POINT_XYZ ECEFtoBLH(POINT_XYZ PointXYZ)
{
    POINT_XYZ coordinatesBLH;
    
    double Rxy = sqrt(PointXYZ.x*PointXYZ.x + PointXYZ.y*PointXYZ.y);
    double phi = (PointXYZ.z > 0)?(PI/2):(-PI/2);
    double lambda = 0;

    double a = earth_A; //metre
    double f = earth_F;
    double e2 = f*(2-f);
    double high = 0; // metre

    if(Rxy>10e-10)
    {
        double lastNhSinPhi = 0;
        double sinPhi = 0;
        double N = a;
        double NhSinPhi = PointXYZ.z; // means (N + h)*sin(phi)
        double NhCosPhi = Rxy; // means (N + h)*cos(phi)

        while(fabs(NhSinPhi - lastNhSinPhi)>10e-4)
        {
            lastNhSinPhi = NhSinPhi;
            sinPhi = NhSinPhi/sqrt(NhCosPhi*NhCosPhi + NhSinPhi*NhSinPhi);
            N = a/sqrt(1-e2*sinPhi*sinPhi);
            NhSinPhi = PointXYZ.z + N*e2*sinPhi;
        }

        phi = atan2(NhSinPhi,NhCosPhi);
        lambda = atan2(PointXYZ.y,PointXYZ.x);
        high = sqrt(NhSinPhi*NhSinPhi + NhCosPhi*NhCosPhi) - N; // sqrt(NhSinPhi*NhSinPhi + NhCosPhi*NhCosPhi) means (N+h) 
    }

    coordinatesBLH.x = phi;
    coordinatesBLH.y = lambda;
    coordinatesBLH.z = high;

    return coordinatesBLH;
}

Eigen::Vector3d ECEFtoBLH(Eigen::Vector3d PointECEF)
{
    POINT_XYZ PointXYZ = eigenToXYZ(PointECEF);
    POINT_XYZ PointBLH = ECEFtoBLH(PointXYZ);
    Eigen::Vector3d coordinatesBLH(PointBLH.x,PointBLH.y,PointBLH.z);

    return coordinatesBLH;
}

Eigen::Matrix3d matrixECEF2ENU(Eigen::Vector3d pointBLH)
{
    double phi = pointBLH(0);  // base station latitude
    double lambda = pointBLH(1); // base station longitude 

    Eigen::Matrix3d conversionMatrix;
    conversionMatrix(0,0) = -sin(lambda);
    conversionMatrix(0,1) = cos(lambda);
    conversionMatrix(0,2) = 0;

    conversionMatrix(1,0) = -sin(phi)*cos(lambda);
    conversionMatrix(1,1) = -sin(phi)*sin(lambda);
    conversionMatrix(1,2) = cos(phi);

    conversionMatrix(2,0) = cos(phi)*cos(lambda);
    conversionMatrix(2,1) = cos(phi)*sin(lambda);
    conversionMatrix(2,2) = sin(phi);

    return conversionMatrix;
}

POINT_XYZ ENUtoECEF(POINT_XYZ deltaENU,POINT_XYZ baseBLH,POINT_XYZ baseECEF)
{
    Eigen::Vector3d stationBLH(baseBLH.x,baseBLH.y,baseBLH.z);
    Eigen::Matrix3d Cen = matrixECEF2ENU(stationBLH);
    Eigen::Matrix3d Cne = Cen.transpose();

    Eigen::Vector3d deltaPoint(deltaENU.x,deltaENU.y,deltaENU.z);
    Eigen::Vector3d stationPoint(baseECEF.x,baseECEF.y,baseECEF.z);
    Eigen::Vector3d deltaXYZ = stationPoint + Cne * deltaPoint ;
    
    return eigenToXYZ(deltaXYZ);
}

POINT_XYZ ECEFtoENU(POINT_XYZ deltaECEF,POINT_XYZ StationBLH)
{
    POINT_XYZ deltaENU; //Relative origin position in ENU coordinates

    double phi = StationBLH.x;  // base station latitude
    double lambda = StationBLH.y; // base station longitude 

    Eigen::Vector3d deltaXYZ(deltaECEF.x, deltaECEF.y, deltaECEF.z);

    Eigen::Matrix3d conversionMatrix;
    conversionMatrix(0,0) = -sin(lambda);
    conversionMatrix(0,1) = cos(lambda);
    conversionMatrix(0,2) = 0;

    conversionMatrix(1,0) = -sin(phi)*cos(lambda);
    conversionMatrix(1,1) = -sin(phi)*sin(lambda);
    conversionMatrix(1,2) = cos(phi);

    conversionMatrix(2,0) = cos(phi)*cos(lambda);
    conversionMatrix(2,1) = cos(phi)*sin(lambda);
    conversionMatrix(2,2) = sin(phi);

    Eigen::Vector3d pointENU = conversionMatrix*deltaXYZ;

    deltaENU.x = pointENU(0); 
    deltaENU.y = pointENU(1);
    deltaENU.z = pointENU(2);

    return deltaENU;
}

POINT_XYZ getPointENU(POINT_XYZ PointXYZ,POINT_XYZ StationXYZ)
{
    POINT_XYZ StationBLH = ECEFtoBLH(StationXYZ);

    POINT_XYZ deltaECEF;
    deltaECEF.x = PointXYZ.x - StationXYZ.x;
    deltaECEF.y = PointXYZ.y - StationXYZ.y;
    deltaECEF.z = PointXYZ.z - StationXYZ.z;

    return ECEFtoENU(deltaECEF,StationBLH);
}

POINT_XYZ getCoordinatesENU(POINT_XYZ PointXYZ,POINT_XYZ StationXYZ,POINT_XYZ StationBLH)
{
    // POINT_XYZ StationBLH = ECEFtoBLH(StationXYZ);

    POINT_XYZ deltaECEF;
    deltaECEF.x = PointXYZ.x - StationXYZ.x;
    deltaECEF.y = PointXYZ.y - StationXYZ.y;
    deltaECEF.z = PointXYZ.z - StationXYZ.z;

    return ECEFtoENU(deltaECEF,StationBLH);
}

Eigen::Vector3d getCoordinatesENU(Eigen::Vector3d PointXYZ,Eigen::Vector3d StationXYZ,Eigen::Vector3d StationBLH)
{
    POINT_XYZ pointA = eigenToXYZ(PointXYZ);
    POINT_XYZ pointB = eigenToXYZ(StationXYZ);
    POINT_XYZ pointC = eigenToXYZ(StationBLH);

    POINT_XYZ result = getCoordinatesENU(pointA,pointB,pointC);
    Eigen::Vector3d pointENU(result.x,result.y,result.z);

    return pointENU;
}

void curvatureRadius(double latitude,double &Rn,double &Rm)
{
    double phi = latitude;
    double a = earth_A; //metre
    double f = earth_F;
    double e2 = f*(2-f);

    Rn = a/sqrt(1 - e2*sin(latitude)*sin(latitude));  // Radius of curvature of unitary circle [m]
    Rm = Rn*(1 - e2)/(1 - e2*sin(latitude)*sin(latitude));  //  Radius of Meridian Curvature [m]
}

Eigen::Vector3d rotationNen(Eigen::Vector3d position, Eigen::Vector3d velocity, double Rn, double Rm)
{
    Eigen::Vector3d  result;
    result(0) = - velocity(1)/(Rm + position(2) );
    result(1) = velocity(0)/(Rn + position(2));
    result(2) = tan( position(1) )*velocity(1)/( Rn + position(2) );

    return result;
}

Eigen::Vector3d rotationNie(Eigen::Vector3d position)
{
    Eigen::Vector3d result(0,earth_W*cos(position(0)),earth_W*sin(position(0)));

    return result;
}

double gravityAcceleration(Eigen::Vector3d position)
{
    return earth_g*(1 + Beta0*pow(sin(position(0)),2) - Beta1*pow(sin(2*position(0)),2)) - Beta2*position(2);
}

Eigen::Matrix3d rotationMatrix(Eigen::Vector3d vectorR)
{
    Eigen::Matrix3d result(3,3);

    result(0,0) = 0;
    result(0,1) = - vectorR(2);
    result(0,2) = vectorR(1);

    result(1,0) = vectorR(2);
    result(1,1) = 0;
    result(1,2) = - vectorR(0);

    result(2,0) = - vectorR(1);
    result(2,1) = vectorR(0);
    result(2,2) = 0;

    return result;
}

Eigen::Matrix3d transferRotationMatrix(Eigen::Vector3d vectorR)
{
    Eigen::Matrix3d transferMatrix(3,3);
    transferMatrix(0,0) = 1;
    transferMatrix(1,1) = 1;
    transferMatrix(2,2) = 1;

    Eigen::Matrix3d rotation = rotationMatrix( vectorR );

    double deltaSquare = vectorR.transpose()*vectorR;
    double a = 1 - deltaSquare/6 + deltaSquare *deltaSquare/120;
    double b = 0.2 - deltaSquare/24 + deltaSquare *deltaSquare/720;

    return transferMatrix + a* rotation + b * rotation*rotation;
}

void updateQuaternion(double *quaternion, Eigen::Vector3d deltaTheta)
{
	double q1 = quaternion[0];
	double q2 = quaternion[1];
	double q3 = quaternion[2];
	double q4 = quaternion[3];

	double Gx = deltaTheta(0);
	double Gy = deltaTheta(1);
	double Gz = deltaTheta(2);

	double T = 1;

	double Molg = T*sqrt(Gx*Gx+Gy*Gy+Gz*Gz);
	double e = 1-Molg*Molg/8+Molg*Molg*Molg*Molg/384;
	double f = 0.5-Molg*Molg/48;

	double qa = e*q1-f*Gx*T*q2-f*Gy*T*q3-f*Gz*T*q4;
	double qb = f*Gx*T*q1+e*q2+f*Gz*T*q3-f*Gy*T*q4;
	double qc = f*Gy*T*q1-f*Gz*T*q2+e*q3+f*Gx*T*q4;
	double qd = f*Gz*T*q1+f*Gy*T*q2-f*Gx*T*q3+e*q4;

	double qnorm = sqrt(qa*qa+qb*qb+qc*qc+qd*qd);

	quaternion[0] = qa/qnorm;
	quaternion[1] = qb/qnorm;
	quaternion[2] = qc/qnorm;
	quaternion[3] = qd/qnorm;
}

Eigen::Matrix3d angleToMatrix(POINT_XYZ eulerAngle)
{
    double roll = eulerAngle.x;
    double pitch = eulerAngle.y;
    double yaw = eulerAngle.z;

    Eigen::Matrix3d convertMatrix(3,3);
    convertMatrix(0,0) = cos(pitch)*cos(yaw);
    convertMatrix(1,0) = cos(pitch)*sin(yaw);
    convertMatrix(2,0) = -sin(pitch);

    convertMatrix(0,1) = - cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,1) = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,1) = sin(roll)*cos(pitch);

    convertMatrix(0,2) = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,2) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,2) = cos(roll)*cos(pitch);

    return convertMatrix;
}

POINT_XYZ matrixToAngle(Eigen::Matrix3d Cbn)
{
    POINT_XYZ eulerAngle;
    eulerAngle.x = atan2(Cbn(2,1),Cbn(2,2));
    eulerAngle.y = asin(-Cbn(2,0));
    eulerAngle.z = atan2(Cbn(1,0),Cbn(0,0));
    return eulerAngle;
}

POINT_XYZ quaternionToAngle(Eigen::Quaterniond Qbn)
{
    Eigen::Matrix3d Cbn = Qbn.toRotationMatrix();
    return matrixToAngle(Cbn);
}

Eigen::Quaterniond euler2Quaternion(Eigen::Vector3d rotationVector)
{
    // x is roll, y is pitch, z is yaw
    Eigen::AngleAxisd AngleX(rotationVector(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd AngleY(rotationVector(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd AngleZ(rotationVector(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = AngleZ * AngleY * AngleX;
    return q;
}

Eigen::Quaterniond angleToQuaternion(POINT_XYZ eulerAngle)
{
    Eigen::Quaterniond Qbn;
	double Phi = eulerAngle.x;
	double Theta = eulerAngle.y;
	double Psi = eulerAngle.z;

	Qbn.w() = cos(Psi/2)*cos(Phi/2)*cos(Theta/2) + sin(Psi/2)*sin(Phi/2)*sin(Theta/2);
	Qbn.x() = cos(Psi/2)*sin(Phi/2)*cos(Theta/2) - sin(Psi/2)*cos(Phi/2)*sin(Theta/2);
	Qbn.y() = cos(Psi/2)*cos(Phi/2)*sin(Theta/2) + sin(Psi/2)*sin(Phi/2)*cos(Theta/2);
	Qbn.z() = sin(Psi/2)*cos(Phi/2)*cos(Theta/2) + cos(Psi/2)*sin(Phi/2)*sin(Theta/2);

    return Qbn;
}

Eigen::Matrix3d quaternionToMatrix(double *quaternion)
{
	Eigen::Matrix3d Cbn(3,3);

	double q1 = quaternion[0];
	double q2 = quaternion[1];
	double q3 = quaternion[2];
	double q4 = quaternion[3];

	Cbn(0,0) = q1*q1+q2*q2-q3*q3-q4*q4;  //Cbn
	Cbn(0,1) = 2*q2*q3-2*q1*q4;
	Cbn(0,2) = 2*q2*q4+2*q1*q3;

	Cbn(1,0) = 2*q2*q3+2*q1*q4;
	Cbn(1,1) = q1*q1-q2*q2+q3*q3-q4*q4;
	Cbn(1,2) = 2*q3*q4-2*q1*q2;

	Cbn(2,0) = 2*q2*q4-2*q1*q3;
	Cbn(2,1) = 2*q3*q4+2*q1*q2;
	Cbn(2,2) = q1*q1-q2*q2-q3*q3+q4*q4;

	return Cbn;
}
