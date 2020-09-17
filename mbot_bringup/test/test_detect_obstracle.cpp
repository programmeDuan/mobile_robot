#include "mbot_bringup/laser_detect_obstracles.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"find_obstracle");
    ros::NodeHandle nh;

    YATDetectObstracles detect_obstracle;

    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}