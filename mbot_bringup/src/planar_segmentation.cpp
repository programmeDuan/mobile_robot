#include "mbot_bringup/planar_segmentation.h"

YATPlanarSegmentation::YATPlanarSegmentation():plane_error_(0.01),outroi_width_(80),outroi_height_(80),
fx_(424.6),fy_(424.6),cx_(418),cy_(243),element_width_(7),element_height_(7),coordinate_x_(374),
coordinate_y_(380),rect_width_(100),rect_height_(100),imgout_width_(200),imgout_height_(200),
showroi_width_(300),showroi_height_(300),height_threshold_(5)
{
  linepoints_.clear();
}

YATPlanarSegmentation::~YATPlanarSegmentation()
{
  
}

int YATPlanarSegmentation::yatGetElementWidth()
{
    return element_width_;
}

int YATPlanarSegmentation::yatGetElementHeight()
{
    return element_height_;
}

int YATPlanarSegmentation::yatGetHeightThreshold()
{
    return height_threshold_;
}

float YATPlanarSegmentation::yatGetFx()
{
    return fx_;
}

float YATPlanarSegmentation::yatGetFy()
{
    return fy_;
}

float YATPlanarSegmentation::yatGetCx()
{
    return cx_;
}

float YATPlanarSegmentation::yatGetCy()
{
    return cy_;
}

int YATPlanarSegmentation::yatGetCoordinateX()
{
    return coordinate_x_;
}

int YATPlanarSegmentation::yatGetCoordinateY()
{
    return coordinate_y_;
}

int YATPlanarSegmentation::yatGetRectWidth()
{
    return rect_width_;
}

int YATPlanarSegmentation::yatGetRectHeigth()
{
    return rect_height_;
}

Eigen::Matrix4f YATPlanarSegmentation::yatPointToMatrix(double a,double b,double c)
{
    if(c<0)
    {
      c = 0-c;
      b = 0-b;
      a = 0-a;
    }

    double lengthVector = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    double roll = atan2(b,c); // x
    double pitch = - asin(a/lengthVector); // y
    double yaw = 0; // z

    Eigen::Matrix4f convertMatrix(4,4); // Body to Navigation
    convertMatrix(0,0) = cos(pitch)*cos(yaw);
    convertMatrix(1,0) = cos(pitch)*sin(yaw);
    convertMatrix(2,0) = -sin(pitch);

    convertMatrix(0,1) = - cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,1) = cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,1) = sin(roll)*cos(pitch);

    convertMatrix(0,2) = sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
    convertMatrix(1,2) = -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
    convertMatrix(2,2) = cos(roll)*cos(pitch);

    convertMatrix(0,3) =0;
    convertMatrix(1,3) =0;
    convertMatrix(2,3) =0;
    convertMatrix(3,3) =1;
    convertMatrix(3,0) =0;
    convertMatrix(3,1) =0;
    convertMatrix(3,2) =0;

    return convertMatrix;
}

Eigen::Matrix3f YATPlanarSegmentation::yatPointToMatrix3(double a,double b,double c)
{
    if(c<0)
    {
      c = 0-c;
      b = 0-b;
      a = 0-a;
    }
    
    double lengthVector = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    double roll = atan2(b,c); // x
    double pitch = - asin(a/lengthVector); // y
    double yaw = 0; // z

    Eigen::Matrix3f convertMatrix(3,3); // Body to Navigation
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

Eigen::Matrix3f YATPlanarSegmentation::yatGetConvertMatrix()
{
    Eigen::Matrix3f convertMatrix(3,3); // Body to Navigation
    convertMatrix(0,0) = 1/ fx_;
    convertMatrix(0,2) = - cx_/ fx_;
    convertMatrix(1,1) = 1/ fy_;
    convertMatrix(1,2) = - cx_/ fy_;

    convertMatrix(0,1) = 0;
    convertMatrix(1,0) = 0;
    convertMatrix(2,0) = 0;
    convertMatrix(2,1) = 0;
    convertMatrix(2,2) = 1.0;
    return convertMatrix;
}

void YATPlanarSegmentation::yatPlaneSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
              pcl::ModelCoefficients::Ptr coefficients,
              pcl::PointIndices::Ptr planeIndices)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;//创建分割对象
  segmentation.setInputCloud(cloud);             //设置输入点云
  segmentation.setModelType(pcl::SACMODEL_PLANE);//设置模型类型，检测平面
  segmentation.setMethodType(pcl::SAC_RANSAC);//设置方法:随机样本一致性
  segmentation.setDistanceThreshold(plane_error_);//设置距离阈值  表示点到估计模型的距离最大值，
  // segmentation.setMaxIterations(500);
  // segmentation.setDistanceThreshold(0.015);
  segmentation.setOptimizeCoefficients(true);
  segmentation.segment(*planeIndices, *coefficients);//分割操作 存储分割结果到点几何inliers及存储平面模型的系数coefficients
}

void YATPlanarSegmentation::yatProjectPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_cloud,
                   pcl::ModelCoefficients::Ptr coefficients,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);//设置模型类型，检测平面
  proj.setInputCloud (plane);//设置输入点云
  proj.setModelCoefficients (coefficients);//设置模型系数
  proj.filter (*proj_cloud);//得到输出点云
}

void YATPlanarSegmentation::yatRemovePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                 pcl::PointIndices::Ptr cloudIndices,
                 bool setNeg)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setNegative(setNeg);//设置是否提取地面,如果false,提取地面,如果true,提取地面外的物体
  extract.setInputCloud(inputCloud);//设置输入点云
  extract.setIndices(cloudIndices);//设置点索引集合
  extract.filter(*outCloud);//得到输出点云
}

bool YATPlanarSegmentation::yatLoadDepthImage(cv::Mat depth_img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &init_cloud, cv::Rect roi)
{
	//depth_img = depth_img(Rect(0, 240, 848, 240));
	if (depth_img.empty() || depth_img.depth() != CV_16U)
	{
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}
	int rows = depth_img.rows, cols = depth_img.cols;
	int vertex_idx = 0;
	for (int i = 0; i < rows -4 ; i+=4)
	{
		for (int j = 0; j < cols -4; j +=4)
		{
			pcl::PointXYZRGB point;
			point.z = (float)(depth_img.at<unsigned short>(i, j)) / 1000.0;
			//cout<<point.z<< " ";
      //滤掉噪点
			if (point.z > 100000)
			{
				point.x = 0;
				point.y = 0;
				init_cloud->points.push_back(point);
				continue;
			}

			point.x = ((float)j + roi.x - cx_) * point.z / fx_;
			point.y = ((float)i  + roi.y - cy_) * point.z / fy_;
      point.r = 100;
      point.g = 100;
      point.b = 100;
			init_cloud->points.push_back(point);
		}
		//cout<<endl;
	}
	return true;
}
/***********************************************************
函数名称：yatPointCloudToRangeImage
函数功能：将点云信息转化为深度图像
入口参数：input_cloud:输入点云的指针
出口参数：range_image:输出深度图像的指针
备 注：
***********************************************************/
void YATPlanarSegmentation::yatPointCloudToRangeImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,pcl::RangeImage::Ptr range_image)
{
  //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小//
  float angular_resolution = pcl::deg2rad(0.5f);
  //max_angle_width为模拟的深度传感器的水平最大采样角度//
  float max_angle_width = pcl::deg2rad(360.0f);
  //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度//
  float max_angle_height = pcl::deg2rad(180.0f);
  //设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换//
  Eigen::Affine3f sensor_pose (Eigen::Affine3f::Identity ());
  //给传感器的位姿赋值  就是获取点云的传感器的的平移与旋转的向量//
  sensor_pose = Eigen::Affine3f (Eigen::Translation3f (input_cloud->sensor_origin_[0],
                                                             input_cloud->sensor_origin_[1],
                                                             input_cloud->sensor_origin_[2])) *
                        Eigen::Affine3f (input_cloud->sensor_orientation_);
  //深度图像遵循坐标系统//
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  //noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平//
  float noise_level = 0.0;
  //min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区//
  float min_range = 0.0f;
  //border_size  设置获取深度图像边缘的宽度 默认为0//
  int border_size = 1;
  
  //点云信息转化为深度图像//
  range_image->createFromPointCloud(*input_cloud, angular_resolution, max_angle_width, max_angle_height,
                                  sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  std::cout << *range_image << "\n";
}

float YATPlanarSegmentation::yatCaculateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  float sum = 0;
  int valnum = 0;
  for(int i=0; i<cloud->points.size(); i++)
  {
    //cout<<cloud->points[i].z<<endl;
    if(cloud->points[i].z > 0)
    {
      sum += cloud->points[i].z;
      valnum ++;
    }
    
  }
  return sum / valnum;
}

vector<int> YATPlanarSegmentation::yatGetLine(cv::Mat outroi)
{
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy; 
  findContours(outroi,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);//寻找轮廓
  
  // cout<<"countour num is: "<<contours.size()<<endl;
  // imshow("outroi", outroi);
  // waitKey(10);

  vector<int> lines(outroi_height_);

  for(int i=0; i<outroi_height_; i++)
  {
    lines[i] = 0;
  } 

  if(contours.size()==0)
  {
    cout<<"Do not find countour!"<<endl;
    return lines;
  }

  else if(contours.size() == 1)
  {
    cv::Mat showcontimg = cv::Mat::zeros(outroi.size(), CV_8UC1);
    drawContours(showcontimg,contours,0,cv::Scalar(255),CV_FILLED,8,hierarchy);//绘制轮廓
    cv::imshow("contour", showcontimg);

    int valid[outroi_height_] = {0};
    for(int i=0; i<outroi_height_; i++)
    {
      for(int j=0; j<outroi_width_;j++)
      {
        valid[i] += showcontimg.at<uchar>(i, j) / 200;
      }
    }

    for(int i=20; i<outroi_height_; i++)
    {
      lines[i] = valid[i];
    }

    return lines;
  }
  else 
  {
    int index = 0;
    int contourid = -1;
    while(index != -1)
    {
      cv::Rect rect = cv::boundingRect(cv::Mat(contours[index]));//计算轮廓的垂直边界最小矩形

      // cout<<"rect.x="<<rect.x<<endl;
      if(rect.x <= 0)
      {
        contourid = index;
        break;
      }
      else
      {
        index = hierarchy[index][0];
      }
    }
    if(contourid >= 0)
    {
      cv::Mat showcontimg = cv::Mat::zeros(outroi.size(), CV_8UC1);
      drawContours(showcontimg,contours,contourid,cv::Scalar(255),CV_FILLED,8,hierarchy);//绘制轮廓
      cv::imshow("contour", showcontimg);

      int valid[outroi_height_] = {0};
      for(int i=0; i<outroi_height_; i++)
      {
        for(int j=0; j<outroi_width_;j++)
        {
          valid[i] += showcontimg.at<uchar>(i, j) / 255;
        }
      }

      for(int i=20; i<outroi_height_; i+=10)
      {
        lines[i] = valid[i];
      }

      return lines;
    }
  }
}

void YATPlanarSegmentation::yatGetPlaneBoundaryLine(cv::Mat depth,cv::Mat element,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeProjected,
                         pcl::ModelCoefficients::Ptr planeCoef,
                         pcl::PointIndices::Ptr planeIndices)
{
  cv::Mat depth_roi = depth(cv::Rect(coordinate_x_, coordinate_y_, rect_width_, rect_height_));
  yatLoadDepthImage(depth_roi, plane, cv::Rect(coordinate_x_, coordinate_y_, rect_width_, rect_height_));
  /* Perform plane segmentation */
  yatPlaneSeg(plane,planeCoef,planeIndices);//分割平面

  // cout<<"plane_coef "<<plane_coef->values[0]<< "  " <<plane_coef->values[1] << "  "<< plane_coef->values[2]<<endl;
  // cout<<"planeindices "<<planeIndices->indices.size()<<endl;
  Eigen::Matrix4f tmat = yatPointToMatrix(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2]); //求变换矩阵

  yatRemovePlane(planeProjected,plane,planeIndices,false);//提取地面内点
  // yatRemovePlane(planeProjected,plane,planeIndices,true);//提取地面以外的点
  transformPointCloud(*planeProjected, *planeProjected, tmat);//对点云进行刚体变换
  
  float mean = yatCaculateMean(planeProjected) * 100;//计算相机到点云的平均高度

  cout<<"mean is "<<mean<<endl;

  Eigen::Matrix3f ttransmat = (yatPointToMatrix3(planeCoef->values[0], planeCoef->values[1], planeCoef->values[2])) .transpose();//求变换矩阵
  cv::Mat cvtransmat;
  eigen2cv(ttransmat, cvtransmat);//eigen与opencv矩阵转换

  double t1 = cv::getTickCount();

  cv::Mat val= cv::Mat(407040, 3, CV_32F);   

  for(int i = 0; i<depth.rows-2; i +=2)
  {
    for(int j=0; j<depth.cols-2; j +=2)
    {
      float z = val.at<float>(i*depth.cols + j, 2) = (float)(depth.at<unsigned short>(i, j)) / 1000;
      val.at<float>(i*depth.cols + j, 0) = (j - cx_) * z / fx_;
      val.at<float>(i*depth.cols + j, 1) = (i - cy_) * z / fy_;
    }
  }

  cv::Mat outval = val * cvtransmat;
  
  cv::Mat imgout = cv::Mat::zeros(cv::Size(imgout_width_, imgout_height_), CV_8UC1);

  for(int i = 0; i<depth.rows-2; i+=2)
  {
    for(int j = 0; j<depth.cols-2; j+=2)
    {
      int x = outval.at<float>(i*depth.cols + j, 0) * 100;
      int y = outval.at<float>(i*depth.cols + j, 1) * 100;
      float z = outval.at<float>(i*depth.cols + j, 2) * 100;
       
      if(x > -imgout_width_/2 && x < imgout_width_/2 && y <0 && y>-imgout_height_) 
      {
        if(isnan(mean))
            continue;
        else
        {
          if(z < mean + height_threshold_&& z > mean -height_threshold_|| z == 0)
          {
            imgout.at<uchar>(-y, x + imgout_width_/2) = 255;//100
          }

          else
          {
            imgout.at<uchar>(-y, x + imgout_width_/2) = 100;//0.5
          }
        }
      }
    }
  }

  morphologyEx(imgout, imgout, cv::MORPH_CLOSE, element);//闭运算
  cv::Mat outroi = imgout(cv::Rect(100, 0, outroi_width_, outroi_height_)).clone();

  cv::flip(imgout, imgout, 0);
  cv::imshow("show", imgout);

  cv::Mat showroi;
  resize(outroi, showroi, cv::Size(showroi_width_, showroi_height_));

  cv::flip(showroi, showroi, 0);
  cv::imshow("showroi", showroi);
    
  outroi = outroi / 255;
  linepoints_ = yatGetLine(outroi);//提取直线
  cout<<linepoints_.size()<<endl;

  double t2 = (cv::getTickCount() - t1) / cv::getTickFrequency() * 1000;
  cout<<"cost time is: "<< t2 <<endl;

  cv::waitKey(10);
}

