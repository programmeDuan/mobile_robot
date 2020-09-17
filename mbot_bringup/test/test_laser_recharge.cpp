#include "mbot_bringup/laser_recharge.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"test_laser_recharge");
    ros::NodeHandle nh;
    YATLaserRecharge laserrecharge;
    bool task_finished = false;

    ros::Rate r(10);
    // laserrecharge.yatExecute();

    while (ros::ok())
    {
        // if(!task_finished)
        // {
        //     laserrecharge.yatExecute();
        //     if(laserrecharge.yatMotion())
        //     {
        //         task_finished = true;
        //         ros::shutdown();
        //     }
        // }
        laserrecharge.yatExecute();
        // if(laserrecharge.yatGoHomeMotion())
        // {
        //     ros::shutdown();
        // }
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

