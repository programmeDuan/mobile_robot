#include "calib_odom/laser_scan.h"
int main(int argc,char** argv)
{

    ros::init(argc, argv, "message_filter_node");
    ros::Time::init();
    ros::NodeHandle n;
    YATScan scan;

    Odom_calib.yatSetDataLen(12000);
    Odom_calib.yatSetDataZero();
    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    
    // ros::spin();
    return 0;
}