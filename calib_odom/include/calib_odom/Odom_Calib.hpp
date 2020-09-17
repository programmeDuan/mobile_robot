#ifndef ODOM_CALIB_H
#define ODOM_CALIB_H
#include <iostream>
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Householder>
class YATOdomCalib
{

public:
    YATOdomCalib(){
        data_len = 0;
        now_len = 0;
    }

   // virtual ~YATOdomCalib();
    void yatSetDataLen(int len);
    bool yatAddData(Eigen::Vector3d Odom,Eigen::Vector3d scan);
    Eigen::Matrix3d yatSolve();
    bool yatIsFull();
    void yatSetDataZero();

private:
    Eigen::MatrixXd  A;
    Eigen::VectorXd  b;
    int data_len,now_len;
};

#endif
