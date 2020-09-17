#include "mbot_bringup/laser_detect_obstracles.h"
YATDetectObstracles::YATDetectObstracles()
{
    laserscan_sub_ = nh_.subscribe("scan", 1,&YATDetectObstracles::yatScanCallback,this);
    sonar_sub_ = nh_.subscribe("sonar_data", 1,&YATDetectObstracles::yatSonarCallback,this);
    laserdetect_pub_ = nh_.advertise<std_msgs::Bool>("/laser_obstracle",1);
    sonardetect_pub_ = nh_.advertise<std_msgs::Bool>("/sonar_obstracle",1);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}
YATDetectObstracles::~YATDetectObstracles()
{
}

void YATDetectObstracles::yatScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    double angle_min = scan_msg->angle_min;
    double angle_max = scan_msg->angle_max;    
    static double laser2obstracle_dis = 0;//雷达到障碍物的安全距离//
    bool find_obstracle = false;
    std_msgs::Bool msg;

    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
        if(!find_obstracle)
        {
            double range = scan_msg->ranges.at(i);
            double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        
            if (std::isnan(range) || range == 0 || std::isinf(range))
            {
                continue;
            }
            else
            {
            
                if((angle_min < angle && angle < (angle_min +ANGLE_THRESHOLD)) || ((angle_max -ANGLE_THRESHOLD) < angle && angle < angle_max))
                {
                    laser2obstracle_dis = DISTANCE / fabs(cos(angle));
                    // std::cout<<"laser2obstracle_dis="<<laser2obstracle_dis<<std::endl;
                
                    if(range < laser2obstracle_dis  && find_obstracle ==false)
                    {
                        std::cout<<"laser is finding Obstracle!"<<std::endl;
                        msg.data = true;
                        find_obstracle =true;
                        laserdetect_pub_.publish(msg);
                        continue;
                    }
                    else
                    {
                        msg.data = false;
                        // laserdetect_pub_.publish(msg);
                    } 
                }
                else
                {
                    continue;
                }
            }
        }
        else 
        {
            break;
        }
    }
}

void YATDetectObstracles::yatSonarCallback(const geometry_msgs::PoseStampedPtr& sonar_msg)
{
    double distance1 = sonar_msg -> pose.orientation.x;
    double distance2 = sonar_msg -> pose.orientation.y;
    double distance3 = sonar_msg -> pose.orientation.z;
    double distance4 = sonar_msg -> pose.orientation.w;
    std_msgs::Bool msg;

    // std::cout << " distance1 = " << distance1 << std::endl;
    // std::cout << " distance2 = " << distance2 << std::endl;
    // std::cout << " distance3 = " << distance3 << std::endl;
    // std::cout << " distance4 = " << distance4 << std::endl;
    
    if(distance1 < SONAR_DISTANCE || distance2 < SONAR_DISTANCE || distance3 <SONAR_DISTANCE || distance4 <SONAR_DISTANCE)
    {
        msg.data = true;
        std::cout << "sonar is finding obstracle!" << std::endl;
        // sonardetect_pub_.publish(msg);
    }
    else
    {
        msg.data = false;
    }
    sonardetect_pub_.publish(msg);
}

void YATDetectObstracles::yatPublishZeroVelocity()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_pub_.publish(cmd_vel);
}