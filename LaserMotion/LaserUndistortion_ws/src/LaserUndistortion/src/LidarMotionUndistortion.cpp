#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
        laserscan_sub_ = nh_.subscribe("scan",10,&LidarMotionCalibrator::LaserScanCallback,this);
    }


    ~LidarMotionCalibrator()
    {
        // if(tf_!=NULL)
        //     delete tf_;
    }
    void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan_msg)
    {
        ros::Time startTime,endTime;
        startTime = laser_scan_msg->header.stamp;

        sensor_msgs::LaserScan laserScanMsg = *laser_scan_msg;

        //得到最终点的时间//
        int laserbeamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment*laserbeamNum);
        //将雷达数据存储起来//
        std::vector<double>ranges,angles;
        for (int i = 0; i < laserbeamNum; i++)
        {
            double distance = laserScanMsg.ranges[i];
            double angle = laserScanMsg.angle_min + laserScanMsg.angle_increment*i;
            if(distance < 0.15 || std::isnan(distance) || std::isinf(distance))
            {
                distance = laserScanMsg.range_max - 0.001;
            }

            ranges.push_back(distance);
            angles.push_back(angle);
        }

        //转换为pcl::pointcloud for visuailization
        tf::Stamped<tf::Pose> visualPose;
        if(!getLaserPose(visualPose,startTime,tf_))
        {
            ROS_WARN("No visualPose,can not calib!!!!");
            return;
        }
        double visual_yaw = tf::getYaw(visualPose.getRotation());
        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {
            if(ranges[i] < 0.15 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
            {
                continue;
            }

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            pcl::PointXYZRGB pt;
            pt.x = x * cos(visual_yaw) - y * sin(visual_yaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visual_yaw) + y * cos(visual_yaw) + visualPose.getOrigin().getY();
            pt.z = 1.0; 

            unsigned char r = 255, g = 0, b = 0;    //red color 红色是运动畸变去除前的数据
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        //进行矫正
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);


            pcl::PointXYZRGB pt;
            pt.x = x * cos(visual_yaw) - y * sin(visual_yaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visual_yaw) + y * cos(visual_yaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // green color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        //进行显示
        g_PointCloudView.showCloud(visual_cloud_.makeShared());
    }

    // 拿到原始的激光数据来进行处理
    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
    {
        //转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;

        //得到最终点的时间
        int beamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * beamNum);

        // 将数据复制出来
        std::vector<double> angles,ranges;
        for(int i = beamNum - 1; i > 0;i--)
        {   
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angles[i];

            if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
                lidar_dist = 0.0;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        //转换为pcl::pointcloud for visuailization

        tf::Stamped<tf::Pose> visualPose;
        if(!getLaserPose(visualPose, startTime, tf_))
        {

            ROS_WARN("Not visualPose,Can not Calib");
            return ;
        }

        double visualYaw = tf::getYaw(visualPose.getRotation());

        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned char r = 255, g = 0, b = 0;    //red color 红色是运动畸变去除前的数据
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        std::cout << std::endl;

        //进行矫正
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        //转换为pcl::pointcloud for visuailization
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);


            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // green color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        //进行显示
         g_PointCloudView.showCloud(visual_cloud_.makeShared());
    }


    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        // robot_pose.frame_id_ = "base_laser";//雷达坐标系//
        robot_pose.frame_id_ = "laser_link";//修改后的雷达坐标系//
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            // if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            if(!tf_->waitForTransform("/odom", "/laser_link", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
       //获取Ps，Pe
       double base_yaw = tf::getYaw(frame_base_pose.getRotation());
       double base_x = frame_base_pose.getOrigin().getX();
       double base_y = frame_base_pose.getOrigin().getY();

       double start_yaw = tf::getYaw(frame_start_pose.getRotation());//Ps
       double start_x = frame_start_pose.getOrigin().getX();
       double start_y = frame_start_pose.getOrigin().getY();

       double end_yaw = tf::getYaw(frame_end_pose.getRotation());//Pe
       double end_x = frame_end_pose.getOrigin().getX();
       double end_y = frame_end_pose.getOrigin().getY();

       for(int i = 1; i < beam_number; i++)
       {
           //二次插值 y = y0 + (y1-y0)/(x1-x0)*(x-x0) Pm = LinearInterp(Ps,Pe,m-s/e-s);
           // start_yaw 时 i =1； end_yaw时 i=beam_number
           //求解的机器人位姿//
           double interp_yaw = start_yaw + (end_yaw - start_yaw) / (beam_number - 1) * i;
           double interp_x = start_x + (end_x - start_x) / (beam_number - 1) * i;
           double interp_y = start_y + (end_y - start_y) / (beam_number - 1) * i;
           
           //求激光转化之前的激光坐标系下的坐标//
           double x = ranges[startIndex + i] * cos(angles[startIndex + i]);
           double y = ranges[startIndex + i] * sin(angles[startIndex + i]);
           
           //根据求解的位姿把所有激光点转换到同一坐标系下//
           //R << cos -sin
           //     sin cos
           double odom_x = x * cos(interp_yaw) - y * sin(interp_yaw) + interp_x;
           double odom_y = x * sin(interp_yaw) + y * cos(interp_yaw) + interp_y;
           //Psi 
           double delta_x = odom_x - base_x; 
           double delta_y = odom_y - base_y;
           x = delta_x * cos(base_yaw) + delta_y * sin(base_yaw);
           y = -delta_x * sin(base_yaw) + delta_y * cos(base_yaw);
           ranges[i] = std::sqrt(x*x+y*y);
           angles[i] = std::atan2(y,x);
       }
    }



    //激光雷达数据　分段线性进行插值　分段的周期为10ms
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //统计激光束的数量
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; // 每束激光数据的时间间隔

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        //基准坐标就是第一个位姿的坐标
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //分段线性,时间段的大小为interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //得到起点和终点的位姿
                //终点的位姿
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //对当前的起点和终点进行插值
                //interpolation_time_duration中间有多少个点.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                //更新时间
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber laserscan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);
    // tf::TransformListener *tf = NULL;
    // ros::Duration(10.0).sleep();
    // LidarMotionCalibrator tmpLidarMotionCalib(tf);

    ros::spin();
    return 0;
}


