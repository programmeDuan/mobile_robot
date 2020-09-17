#include <opencv2/opencv.hpp>
#include "opencv2/video/background_segm.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include<vector>
#include<algorithm>
#include "mbot_bringup/vibe_plus.h"

using namespace std;
using namespace cv;

//切割子图的数量为128*128
#define CUTCOLS 9
#define CUTROWS 9

//图片的读取,写入,保存路径//
string write_path = "/home/yat/图片/6-6/";
string read_path  = "/home/yat/图片/6-6/*.jpg";
string save_path = "/home/yat/图片/6-4.2/";
string video_path = "/home/yat/视频/Webcam/2020-06-08-154040.webm";

//数字量转变成字符串量子函数//
string yatNum2Str(int i)
{
	
	stringstream ss;
	ss << i;
	return ss.str();
}

//求平均值//
double yatCalMean(double array[],int size)
{
    double sum = 0;

    for(int i = 0; i<size;i++)
    {
        sum+=array[i];
    }

    cout<<"average::"<<sum/size<<endl;
    return sum/size;
}
//图片采集//
void yatImageAcpuisition()
{
    VideoCapture capture(1);
    string name;
    namedWindow("Image", CV_WINDOW_AUTOSIZE);
    int i=0;
    while (1)
    {
        Mat frame;
        capture >> frame;
        if (32 == waitKey(20))//空格拍照
        {	
            name = write_path + to_string(i)+".jpg";
            imwrite(name, frame);
            cout << name << endl;
            i++;
        }
        if (97 == waitKey(10))//'a'退出
        {
            break;
        }
        imshow("Image",frame);
  
   }
}
//模糊检测，如果原图像是模糊图像，返回0，否则返回1//  
bool yatBlurDetect(Mat srcImage, double &blurPer)
{
    Mat srcBlur, gray1, gray2, gray3, dstImage; 
    double thre = 300; //控制阈值//
    //pyrDown(srcImage, dstImage, Size(srcImage.cols/2, srcImage.rows/2));
    //高斯滤波//
    GaussianBlur(srcImage, srcBlur, Size(3, 3), 0, 0, BORDER_DEFAULT); 
    //imshow("sas", srcBlur);
    //imshow("ssas", srcImage);
    //使用线性变换转换输入数组元素成8位无符号整型 归一化为0-255//
    convertScaleAbs(srcBlur, srcImage); 
    // cout<<"srcImage.channels::"<<srcImage.channels()<<endl;

    if (srcImage.channels() != 1)  
    {  
        //进行灰度化//  
        cvtColor(srcImage, gray1, CV_BGR2GRAY);  
    }
    else  
    {  
        gray1 = srcImage.clone();  
    } 

    imshow("gray1",gray1);

    // gray1 = yatImagePreprocess(srcImage);
    // cv::medianBlur(gray1,dstImage,3);

    Mat tmp_m1, tmp_sd1;    //用来存储均值和方差  
    double m1 = 0, sd1 = 0, meanValue = 0.0,sobelmeanValue=0.0;  
    //使用3x3的Laplacian算子卷积滤波//  
    // Laplacian(gray1, gray2, CV_16S, 3, 1, 0, BORDER_DEFAULT);
    Laplacian(gray1, gray2, CV_16U, 3, 1, 0, BORDER_DEFAULT);
    
    Sobel(gray1, gray3, CV_16U, 1, 1);
    //double minVal, maxVal;
    //minMaxLoc(gray2, &minVal, &maxVal);
    //double alpha = 255 / (maxVal - minVal), beta = -255 * minVal / (maxVal - minVal);*

    //归到0~255//  
    // convertScaleAbs(gray2, gray3);
    //imshow("Lap", gray3);
    //计算均值和方差//  
    meanStdDev(gray1, tmp_m1, tmp_sd1);
    meanValue = mean(gray2)[0];
    sobelmeanValue = mean(gray3)[0];


    cout<<"meanValue::"<<meanValue<<endl;
    // cout<<"sobelmeanValue::"<<sobelmeanValue<<endl;
    //imshow("Lap后", gray3);
    //waitKey(0);
    m1 = tmp_m1.at<double>(0, 0);     //均值//  
    sd1 = tmp_sd1.at<double>(0, 0);       //标准差//  
    //cout << "原图像：" << endl;  
    //cout << "均值: " << m1 << " , 方差: " << sd1*sd1 << endl; 
    blurPer = sd1*sd1; //方差

    // cout<<"blurPer::"<<blurPer<<endl;
    //waitKey(0);
    if (blurPer < thre)
    {
        return false; 
    }  
    else
    {
        return true;
    }   
}

int yatGetVideoCapture(string path,Mat frame)
{
    cv::VideoCapture capture;
    capture.open(path);
    // capture.set(CV_CAP_PROP_FRAME_WIDTH,160);
    // capture.set(CV_CAP_PROP_FRAME_HEIGHT,120);
    if(!capture.isOpened())
    {
        cout<<"加载视频失败，请检查文件路径设置！"<<endl;
        return -1;
    }
    // namedWindow("mask",CV_WINDOW_AUTOSIZE);
    namedWindow("frame",WINDOW_AUTOSIZE);
    // namedWindow("background", WINDOW_AUTOSIZE);
    Mat segModel,updateModel;
    // Mat mask;
    // Mat background;
    // Mat temp_frame,current_frame,previous_frame,temp2,target_frame;
    // bool flag = false;
    int delay = 1000/capture.get(CV_CAP_PROP_FPS);

    // 程序运行时间统计变量//
    // the Time Statistical Variable of Program Running Time//
    double time;
    double start;

    YATViBePlus vibeplus;
    bool count = true;

    // Ptr<BackgroundSubtractorMOG2> bgSubtractor = createBackgroundSubtractorMOG2(1,16,false);
    while (1)
    {
       capture>>frame;
    //    temp_frame = frame;
       
       if( !capture.read(frame) )
	   {
		   cout<<"从视频中读取图像失败或者读完整个视频"<<endl;
		   return -2;
	   }

       imshow("frame",frame);     

       //vibe算法//
       //捕获图像//
       vibeplus.yatFrameCapture(frame);

       start = static_cast<double>(getTickCount());
       vibeplus.yatRun();
       time = ((double)getTickCount() - start) / getTickFrequency() * 1000;
       cout << "Time of Update ViBe+ Background: " << time << "ms"<<endl;

       segModel = vibeplus.yatGetSegModel();
       updateModel = vibeplus.yatGetUpdateModel();
       imshow("segModel",segModel);
       imshow("updateModel",updateModel);
       if(waitKey(delay)==27)
       {
           break;
       }

      
       
    //    //帧差法//
    //    if(flag==false)
    //    {
    //        cvtColor(temp_frame,previous_frame,CV_BGR2GRAY);
    //     //    previous_frame = frame.clone();
    //        flag =true;
    //    }
    //    else
    //    {
    //        cvtColor(temp_frame,current_frame,CV_BGR2GRAY);
    //        absdiff(current_frame,previous_frame,target_frame);
    //        //absdiff(current_frame,temp2,target_frame);
    //        threshold(target_frame,target_frame,80,255,CV_THRESH_BINARY);
    //        imshow("target_frame",target_frame);
    //        waitKey(delay);
           
    //    }
       
    //背景差法//
    //    bgSubtractor->operator()(frame,mask,0.001);
    //    bgSubtractor->apply(frame,mask,-1);
    //    bgSubtractor->getBackgroundImage(background);
    //    imshow("background",background);
	//    imshow("mask",mask);
	//    waitKey(delay);

    }
    // delete bgSubtractor;
    capture.release();
}

//图像预处理:平均值法将图像灰度化并进行中值滤波//
cv::Mat yatImagePreprocess(cv::Mat srcImage)
{
    int height = srcImage.rows;//行数
    int width = srcImage.cols;//列数
    // Mat tmpImage = srcImage.clone();
    Mat gray = cv::Mat::zeros(cv::Size(width,height),CV_8UC1);
    // Mat gray(cv::Size(width,height),CV_8UC1);
     
    // Mat gray;
    // cvtColor(srcImage,gray,CV_BGR2GRAY);
    
    //平均值将图像灰度化//
    for (int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            if(srcImage.channels() ==3)
            {
                int r = (int)srcImage.at<Vec3b>(i,j)[0];
                int g = (int)srcImage.at<Vec3b>(i,j)[1];
                int b = (int)srcImage.at<Vec3b>(i,j)[2];
                int temp = (r+g+b)/3;

                gray.at<uchar>(i,j) = temp;
            } 
        }
    }

    Mat outImage;
    // imshow("gray",gray);

    //中值滤波//
    cv::medianBlur(gray,outImage,5);

    // imshow("outImage",outImage);
    return outImage;
}

//单幅图像信息熵计算//
double yatCalEntropy(Mat img)
{
    //创建数组并初始化//
    double temp[256] = {0.0};

    // 计算每个像素的累积值//
    for (int i = 0; i<img.rows; i++)
    {
        // 有效访问行列的方式//
        const uchar* t = img.ptr<uchar>(i);
        for (int j = 0; j<img.cols; j++)
        {
            int tmp = t[j];
            temp[tmp] = temp[tmp] + 1;
        }
    }

    // 计算每个像素的概率//
    for (int i = 0; i<256; i++)
    {
        temp[i] = temp[i] / (img.rows*img.cols);
    }

    double result = 0;
    // 计算图像信息熵//
    for (int i = 0; i<256; i++)
    {
        if (temp[i] == 0.0)
        {
            result = result;
        } 
        else
        {
            result = result - temp[i] * (log(temp[i]) / log(2.0));
        }    
    }
    // cout <<"entropyresult::"<<result<<endl;
    return result;
}

// 两幅图像联合信息熵计算//
double yatCalComEntropy(Mat img1, Mat img2, double img1_entropy, double img2_entropy)
{
    double temp[256][256] = {0.0};

    // 计算联合图像像素的累积值//
    for (int m1 = 0, m2 = 0; m1 < img1.rows, m2 < img2.rows; m1++, m2++)
    {    // 有效访问行列的方式
        const uchar* t1 = img1.ptr<uchar>(m1);
        const uchar* t2 = img2.ptr<uchar>(m2);
        for (int n1 = 0, n2 = 0; n1 < img1.cols, n2 < img2.cols; n1++, n2++)
        {
            int i = t1[n1], j = t2[n2];
            temp[i][j] = temp[i][j] + 1;
        }
    }

    // 计算每个联合像素的概率//
    for (int i = 0; i < 256; i++)
    {
        for (int j = 0; j < 256; j++)

        {
            temp[i][j] = temp[i][j] / (img1.rows*img1.cols);
        }
    }

    double result = 0.0;
    //计算图像联合信息熵//
    for (int i = 0; i < 256; i++)
    {
        for (int j = 0; j < 256; j++)

        {
            if (temp[i][j] == 0.0)
                result = result;
            else
                result = result - temp[i][j] * (log(temp[i][j]) / log(2.0));
        }
    }

    //得到两幅图像的互信息熵//
    img1_entropy = yatCalEntropy(img1);
    img2_entropy = yatCalEntropy(img2);
    result = img1_entropy + img2_entropy - result;

    cout<<"ComEntropyresult::"<<result<<endl;

    return result;
}

//计算切割后所有小的图像的平均信息熵//
void yatImageCut(Mat img)
{
    //创建一个容器存储切割后的子图//
    vector<Mat> ceil_img;
    int height = img.rows;
	int width = img.cols;
    
    //子图的尺寸//
    int ceil_height = (int)(height / CUTROWS);
	int ceil_width = (int)(width / CUTCOLS);

	int ceil_down_height = height - (CUTROWS - 1)*ceil_height;
	int ceil_right_width = width - (CUTCOLS - 1)*ceil_width;
    
    //切割图像并将子图存储在容器中//
    for(int i =0; i<CUTROWS-1;i++)
    {
        for (int j = 0; j<CUTCOLS; j++)
		{
			if (j<CUTCOLS - 1)
			{
				Rect rect(j*ceil_width, i*ceil_height, ceil_width, ceil_height);
				ceil_img.push_back(img(rect));
 
			}
			else
			{
				Rect rect((CUTCOLS - 1)*ceil_width, i*ceil_height, ceil_right_width, ceil_height);
				ceil_img.push_back(img(rect));
			}
		}
    }

    for (int i = 0; i<CUTCOLS; i++)
	{
		if (i<CUTCOLS - 1)
		{
			Rect rect(i*ceil_width, (CUTROWS - 1)*ceil_height, ceil_width, ceil_down_height);
			ceil_img.push_back(img(rect));
		}
		else   //右下角这个图像块//
		{
			Rect rect((CUTCOLS - 1)*ceil_width, (CUTROWS - 1)*ceil_height, ceil_right_width, ceil_down_height);
			ceil_img.push_back(img(rect));
            
		}
	}
    
    // cout << "分块个数：" << ceil_img.size() << endl;

    Mat dst;
    int array_size = ceil_img.size();
    double score =0;
    double temp[array_size]={0};
    // string name;
 
	for (int i = 0; i < ceil_img.size(); i++)
	{
        
		dst = ceil_img[i];
        temp[i]=yatCalEntropy(dst);

        // 利用分割后的所有小图像来进行模糊检测得到的信息不准确//
        // yatBlurDetect(dst,score);
        // temp[i]=score;

        // name = save_path +yatNum2Str(i+1)+ ".jpg";
		// imwrite(name, dst);
	}
    //计算整张图像切割后的所有小图像的信息熵平均值//
    yatCalMean(temp,array_size);
}

int main(int argc, char** argv)
{
    Mat frame;
    yatGetVideoCapture(video_path,frame);
    // yatImageAcpuisition();
    // double score = 0;
    // std::vector<cv::String> image_files;
	// cv::glob(read_path, image_files);
	// if (image_files.size() == 0) 
    // {
	// 	std::cout << "No image files[jpg]" << std::endl;
	// 	return 0;
	// }

    // for (unsigned int i = 0; i < image_files.size(); i++) 
    // {
    //     printf("**********第%d张图片************\n",i+1);
	// 	Mat image = cv::imread(image_files[i]);

    //     // Mat outImage = yatImagePreprocess(image);
    //     // yatImageCut(outImage);
    //     yatBlurDetect(image,score);
    //     // cout<<"socre::"<<score<<endl;

	// 	imshow("image", image);
	// 	waitKey(1000);

	// }
	return 0;
}
