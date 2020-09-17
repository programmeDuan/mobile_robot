#include "mbot_bringup/obstacledet.h"

// void YATObstracleDet::YATObstracleDet(const int distance_threshold,const int thresh,const int max_thresh,const int area):
// distance_threshold_(distance_threshold),thresh_(thresh),max_thresh_(max_thresh),area_(area)
// {
//     result_.clear();
// }

YATObstracleDet::YATObstracleDet():distance_threshold_(1000),thresh_(20),max_thresh_(255),area_(500),camera_height_(56.0)
{
    result_.clear();
}
YATObstracleDet::~YATObstracleDet()
{
}

int YATObstracleDet::yatGetDistanceThresh()
{
    return distance_threshold_;
}

int YATObstracleDet::yatGetThresh()
{
    return thresh_;
}

int YATObstracleDet::yatGetMaxThresh()
{
    return max_thresh_;
}

int YATObstracleDet::yatGetArea()
{
    return area_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  YATObstracleDet::yatGetObstraclePointCloud(cv::Mat depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeProjected,
                         pcl::ModelCoefficients::Ptr planeCoef,
                         pcl::PointIndices::Ptr planeIndices,
                         YATPlanarSegmentation *planarseg)
{
    planarseg->yatLoadDepthImage(depth,plane,cv::Rect(0, 0, 848, 480));
    planarseg->yatPlaneSeg(plane,planeCoef,planeIndices);//分割平面

    Eigen::Matrix4f tmat = planarseg->yatPointToMatrix(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2]); //求变换矩阵

    transformPointCloud(*plane, *plane, tmat);//对点云进行刚体变换
    planarseg->yatRemovePlane(planeProjected,plane,planeIndices,true);//提取地面外的点
    // planarseg->yatRemovePlane(planeProjected,plane,planeIndices,false);
   
  
    float mean = planarseg->yatCaculateMean(plane) * 100;//计算相机到点云的平均距离

    // cout<<"mean is "<<mean<<endl;

    if(mean > camera_height_+planarseg->yatGetHeightThreshold())
    {
        cout<<"PointClond transform failed, the mean data is unreasonable and should be discarded"<<endl;
        planeProjected->resize(0);

        return planeProjected;
    }

    else
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>); 

        for(int i = 0;i < planeProjected->points.size(); i++)
        {
            if(mean < (planeProjected->points.at(i).z*100+planarseg->yatGetHeightThreshold()) || planeProjected->points.at(i).z == 0)
            {
                continue;
            }

            else
            {
                // cout<<"x::"<<planeProjected->points.at(i).x<<endl;
                // cout<<"y::"<<planeProjected->points.at(i).y<<endl;
                if(fabs(planeProjected->points.at(i).x) *100 > 50|| fabs(planeProjected->points.at(i).y) * 100 > 100)
                {
                    continue;
                }
                else
                {
                    cloud_projected->points.push_back(planeProjected->points.at(i));
                }
                // cloud_projected->points.push_back(planeProjected->points.at(i));
            }
         }
        //  cout<<"cloud_projected->points.size()::"<<cloud_projected->points.size()<<endl;         
         return cloud_projected;
     }
    // Eigen::Matrix3f ttransmat = (planarseg->yatPointToMatrix3(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2])) .transpose();//求变换矩阵
    
    // cv::Mat cvtransmat;
    // eigen2cv(ttransmat, cvtransmat);//eigen与opencv矩阵转换

    // cv::Mat val= Mat(407040, 3, CV_32F);   

    // for(int i = 0; i<depth.rows-2; i +=2)
    // {
    //     for(int j=0; j<depth.cols-2; j +=2)
    //     {
    //         float z = val.at<float>(i*depth.cols + j, 2) = (float)(depth.at<unsigned short>(i, j)) / 1000;
    //         val.at<float>(i*depth.cols + j, 0) = (j - planarseg->yatGetCx()) * z / planarseg->yatGetFx();
    //         val.at<float>(i*depth.cols + j, 1) = (i - planarseg->yatGetCy()) * z / planarseg->yatGetFy();
    //     }
    // }

    // // val.convertTo(val,CV_8UC1,1.0/16);
    // // cv::imshow("val",val);
    // cv::Mat outval = val * cvtransmat;//

    // cv::Mat depthout;
    // depth.copyTo(depthout);

    // for(int i = 0; i < depth.rows-2; i+=2)
    // {
    //     for(int j = 0; j < depth.cols-2; j+=2)
    //     {
    //         float z = outval.at<float>(i*depth.cols + j, 2) * 100;
    //         if(z < mean + planarseg->yatGetHeightThreshold() && z > mean - planarseg->yatGetHeightThreshold() || z == 0)
    //         // if(z > mean - planarseg->yatGetHeightThreshold() || z == 0)
    //         {
    //             depthout.at<unsigned short>(i,j) = 0;
    //         }
    //     }
    // } 

    // cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    // morphologyEx(depthout,depthout,MORPH_CLOSE,element);

    // return depthout; 
}

bool YATObstracleDet::yatIsFindObstracle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
    if(pointcloud->points.size() < 50 && pointcloud->points.size() > 0)
    {
        return false;
    }

    else if(pointcloud->points.size() == 0)
    {
        return true;
    }

    else
    {
        for(int i = 0; i < pointcloud->points.size(); i++)
        {
            // cout<<"fabs(pointcloud->points.at(i).y*100)"<<fabs(pointcloud->points.at(i).y*100)<<endl;
            // cout<<"fabs(pointcloud->points.at(i).x*100)"<<fabs(pointcloud->points.at(i).x*100)<<endl;
            if(fabs(pointcloud->points.at(i).y*100) < 60 && fabs(pointcloud->points.at(i).x*100) < 25)
            {
                cout<<"Finding obstracle,please remove the obstracle"<<endl;
                return true;
            }

            else
            {
                continue;
            }
        }

        return false;   
    }  
}

cv::Mat YATObstracleDet::yatPointCloudToDepth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{

}

void YATObstracleDet::yatMaskDepth(cv::Mat depth,cv::Mat outimg,int threshold)
{
    int outimg_rows = depth.rows;
    int outimg_cols = depth.cols;
    for(int i = 0; i < outimg_rows; i++)
    {
        for(int j = 0; j<outimg_cols; j++)
        {
            if(depth.at<unsigned short>(i,j) > threshold)
            {
                outimg.at<unsigned short>(i,j) = 0;
            }
        }
    } 
}

vector<vector<cv::Point>> YATObstracleDet::yatFindObstacle(cv::Mat depth,int thresh,int max_thresh,int area)
{
    cv::Mat depthimg;
    depth.copyTo(depthimg);
    yatMaskDepth(depth,depthimg,600);//深度图二值化
    depthimg.convertTo(depthimg,CV_8UC1,1.0/16);
    cv::imshow("depthimg",depthimg);

    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::Mat outimg;
    morphologyEx(depthimg,outimg,cv::MORPH_OPEN,element);
    cv::imshow("outimg",outimg);

    cv::Mat depth_copy = depthimg.clone();
    cv::Mat threshold_outimg;
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::RNG rng(12345);

    //Binarize the image
    threshold(depthimg,threshold_outimg,thresh,max_thresh,CV_THRESH_BINARY);
    cv::imshow("threshold_outimg",threshold_outimg);

    //find contours
    findContours(threshold_outimg,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,cv::Point(0, 0));
  
    //Calculate its convex hull for each contour
    vector<vector<cv::Point>> hull(contours.size());
    vector<vector<cv::Point>> result;
    for(int i = 0; i < contours.size();i++)
    {
        convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    //Draw the contour and its convex hull
    cv::Mat drawimg =cv::Mat::zeros(threshold_outimg.size(),CV_8UC3);

    //cout<<"threshold_outimg.size()::"<<threshold_outimg.size()<<endl;//640*480
    for(int i = 0; i < contours.size(); i++)
    {
        // cout<<"contourArea(contours[i]::"<<contourArea(contours[i])<<endl;
        if(contourArea(contours[i]) < area)
        {
            continue;//小于最小有效面积的凸包不予考虑
        }    
        result.push_back(hull[i]);
        cv::Scalar color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
        drawContours(drawimg,contours,i,color,1,8,vector<cv::Vec4i>(),0,cv::Point());//绘制轮廓   
        drawContours(drawimg,hull,i,color,1,8,vector<cv::Vec4i>(),0,cv::Point()); //绘制凸包  
    }

    cv::imshow("contours", drawimg);
    cv::waitKey(10);

    return result;
}