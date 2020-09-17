#include "mbot_bringup/move_along_edge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "movealongedge");
    ros::NodeHandle nh;

    YATMoveAlongEdge movealongedge;

    ros::Rate r(10);
    while(ros::ok())
    {
        movealongedge.yatExecute();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}