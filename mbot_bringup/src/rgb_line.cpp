#include "rgb_line.hpp"


YatProcRgbLine::~YatProcRgbLine()
{

}

/***********************************************************
函数名称：yatFilterLines
函数功能：卷积核滤波处理图像
入口参数：inImage: 输入的图像
出口参数：outImage： 输出滤波后的图像
备 注：
***********************************************************/
void YatProcRgbLine::yatFilterLines(cv::Mat inImage, cv::Mat &outImage)
{
    //定义两个卷积核
    cv::Mat fx;
    cv::Mat fy;

    float derivp[] = {1.000000e-16, 1.280000e-14, 7.696000e-13, 2.886400e-11, 7.562360e-10, 1.468714e-08, 2.189405e-07, 2.558828e-06, 2.374101e-05, 1.759328e-04, 1.042202e-03, 4.915650e-03, 1.829620e-02, 5.297748e-02, 1.169560e-01, 1.918578e-01, 2.275044e-01, 1.918578e-01, 1.169560e-01, 5.297748e-02, 1.829620e-02, 4.915650e-03, 1.042202e-03, 1.759328e-04, 2.374101e-05, 2.558828e-06, 2.189405e-07, 1.468714e-08, 7.562360e-10, 2.886400e-11, 7.696000e-13, 1.280000e-14, 1.000000e-16};
    int derivLen = 33; //23; 13; 33;
    float smoothp[] = {-1.000000e-03, -2.200000e-02, -1.480000e-01, -1.940000e-01, 7.300000e-01, -1.940000e-01, -1.480000e-01, -2.200000e-02, -1.000000e-03};
    int smoothLen = 9; //9; 17;

    fy = cv::Mat(1, smoothLen, CV_32F, smoothp);
    fx = cv::Mat(derivLen, 1, CV_32F, derivp);

    //减去均值
    cv::Scalar mean = cv::mean(inImage);
    cv::subtract(inImage, mean, outImage);

    //计算卷积
    cv::filter2D(outImage, outImage, outImage.depth(), fx);
    cv::filter2D(outImage, outImage, outImage.depth(), fy);

    //cv::imshow("subIMage", outImage*50);
}


/***********************************************************
函数名称：yatGetQuantile
函数功能：获取前qtile比例的值
入口参数：mat: 输入的图像
        qtile:比例值
出口参数：输出的值
备 注：
***********************************************************/
float YatProcRgbLine::yatGetQuantile(const cv::Mat mat, float qtile)
{
    //转换成行向量
    cv::Mat rowMat = mat.reshape(0, 1);


    //std::cout<<"reshape cols is: "<<rowMat.cols<<std::endl;
    
    float qval = quantile((float *)rowMat.data, rowMat.cols, qtile);

    return qval;
}

/***********************************************************
函数名称：yatGetMatLocalMax
函数功能：用r和theta获取线段在直线上的端点
入口参数：r:r
        theta:theta
        bbox:端点的区域
出口参数：outlines:输出的线段端点
备 注：
***********************************************************/
void YatProcRgbLine::yatGetMatLocalMax(const cv::Mat inMat, vector<double> &localMaxima, vector<cv::Point> &localMaximaLoc, double threshold)
{
    double val;
    for(int i=1; i<inMat.rows-1; i++)
    {
        for (int j=1; j<inMat.cols-1; j++)
        {
            val = inMat.at<ushort>(i, j); 
            //判断是否当前值比周围值都大
            if( val > inMat.at<ushort>(i-1, j-1) && 
            val > inMat.at<ushort>(i-1, j)  && 
            val > inMat.at<ushort>(i-1, j+1)  && 
            val > inMat.at<ushort>(i, j-1)  && 
            val > inMat.at<ushort>(i, j+1)  && 
            val > inMat.at<ushort>(i+1, j-1) && 
            val > inMat.at<ushort>(i+1, j) && 
            val > inMat.at<ushort>(i+1, j+1) && val >= threshold) 
            { 
                vector<double>::iterator k;
                vector<cv::Point>::iterator l;

                /*升序插入*/
                for(k=localMaxima.begin(), l=localMaximaLoc.begin(); k != localMaxima.end()  && val<= *k; k++,l++);

                localMaxima.insert(k, val);
                localMaximaLoc.insert(l, cvPoint(j, i));
            }
        }

    }
}

/***********************************************************
函数名称：yatIsPointInside
函数功能：用r和theta获取线段在直线上的端点
入口参数：r:r
        theta:theta
        bbox:端点的区域
出口参数：outlines:输出的线段端点
备 注：
***********************************************************/
bool YatProcRgbLine::yatIsPointInside(cv::Point2f point, CvSize bbox)
{
  return (point.x>=0 && point.x<=bbox.width && point.y>=0 && point.y<=bbox.height) ? true : false;
}

/***********************************************************
函数名称：yatIntersectLineRThetaWithBB
函数功能：用r和theta获取线段在直线上的端点
入口参数：r:r
        theta:theta
        bbox:端点的区域
出口参数：outlines:输出的线段端点
备 注：
***********************************************************/
void YatProcRgbLine::yatIntersectLineRThetaWithBB(float r, float theta, const cv::Size bbox, YatLine *outLine)
{
    double xup, xdown, yleft, yright;

    //上边界和下边界: y=0 和 y=bbox.height-1
    if (cos(theta)==0) //水平线
    {
        xup = bbox.width+2;
        xdown = bbox.width+2;
    }
    else
    {
        xup = r / cos(theta);
        xdown = (r-bbox.height*sin(theta))/cos(theta);
    }

    //左边界和右边界: x=0 和 x=bbox.widht-1
    if (sin(theta)==0) //水平线
    {
        yleft = yright = bbox.height+2;
    }
    else
    {
        yleft = r/sin(theta);
        yright = (r-bbox.width*cos(theta))/sin(theta);
    }

    cv::Point2f pts[4] = {{xup, 0},{xdown,bbox.height}, {0, yleft},{bbox.width, yright}};

    //起始点
    int i;
    for (i=0; i<4; i++)
    {
        //点是否在图像里
        if(yatIsPointInside(pts[i], bbox))
        {
            outLine->startPoint.x = pts[i].x;
            outLine->startPoint.y = pts[i].y;
            break;
        }
    }

    //结束点
    for (i++; i<4; i++)
    {
        //点是否在图像里
        if(yatIsPointInside(pts[i], bbox))
        {
            outLine->endPoint.x = pts[i].x;
            outLine->endPoint.y = pts[i].y;
            break;
        }
    }
}

/***********************************************************
函数名称：yatGetHoughTransformLines
函数功能：用hough变换获取图像上的直线
入口参数：inImage
出口参数：inlines:提取到的直线
        lineScores:对应直线的分数
备 注：
***********************************************************/
void YatProcRgbLine::yatGetHoughTransformLines(cv::Mat inImage, vector <YatLine> *lines, vector <float> *lineScores)
{
    float rMin = inImage.rows * 0.05;
    float rMax = inImage.cols;
    float rStep = 1;                           
    float thetaMin = -20 * CV_PI / 180;
    float thetaMax = 20 * CV_PI / 180;
    float thetaStep = 1 * CV_PI / 180;
    bool binarize = true;
    bool localMaxima = true;

    float detectionThreshold = inImage.rows / 3;
    bool smoothScores = false;
    bool group = true;
    float groupThreshold = 20;

    cv::Mat image = inImage.clone();

    //定义霍夫空间矩阵
    int rBins = int((rMax-rMin)/rStep);
    int thetaBins = int((thetaMax-thetaMin)/thetaStep);
    cv::Mat houghSpace = cv::Mat::zeros(rBins, thetaBins, CV_16UC1); //float_MAT_TYPE);

    //初始化r和theta的值
    float *rs = new float[rBins];
    float *thetas = new float[thetaBins];
    float r, theta;
    int ri, thetai;
    for (r=rMin+rStep/2,  ri=0 ; ri<rBins; ri++,r+=rStep)
    {
        rs[ri] = r;
    }
    
    for (theta=thetaMin, thetai=0 ; thetai<thetaBins; thetai++, theta+=thetaStep)
    {
        thetas[thetai] = theta;
    }
    
    //获取图像中的非零值
    int nzCount = cv::countNonZero(image);
    cv::Mat nzPoints = cv::Mat(nzCount, 2, CV_16UC1);
    int idx = 0;
    for (int i=0; i<image.cols; i++)
    {
        for (int j=0; j<image.rows; j++)
        {
            if (image.at<float>(j, i) > 0)
            {
                nzPoints.at<cv::Vec2s>(idx, 0)[0] = i;
                nzPoints.at<cv::Vec2s>(idx, 1)[1] = j;
                idx++;
            }
        }    
    }

    //std::cout<<"r value is: "<<idx<<std::endl;

    //CvMat *rPoints = cvCreateMat(image->width*image->height, thetaBins, CV_32SC1);//float_MAT_TYPE)
    //CvMat *rPoints = cvCreateMat(nzCount, thetaBins, CV_32SC1);//float_MAT_TYPE);
    //cvSet(rPoints, cvRealScalar(-1));
    //loop on x
    //float x=0.5, y=0.5;
    int i, k; 
    for (i=0; i<nzCount; i++)
    {
        for (k=0; k<thetaBins; k++)
        {
            //计算rho 和theta
            theta = thetas[k];
            float rval = nzPoints.at<cv::Vec2s>(i, 0)[0] * cos(theta) + nzPoints.at<cv::Vec2s>(i, 1)[1] * sin(theta); //x y
            int r = (int)( ( rval - rMin) / rStep);
            
            //累加
            if (r>=0 && r<rBins)
            {
                houghSpace.at<ushort>(r, k) ++;
            }
        }
    }

    //std::cout<<"nzcount is: "<<nzCount<<std::endl;
    //std::cout<<houghSpace<<std::endl;

    //平滑
    if (smoothScores)
    {
        cv::GaussianBlur(houghSpace, houghSpace, cv::Size(3, 3), 1);
    }   
        
    vector<double> maxLineScores;
    vector<cv::Point> maxLineLocs;

    //获取局部最大值
    yatGetMatLocalMax(houghSpace, maxLineScores, maxLineLocs, detectionThreshold);

    //获取最大值
    double maxLineScore;
    cv::Point maxLineLoc;
    cv::minMaxLoc(houghSpace, NULL, &maxLineScore, NULL, &maxLineLoc);
    if (maxLineScores.size()==0 && maxLineScore>=detectionThreshold)
    {
        maxLineScores.push_back(maxLineScore);
        maxLineLocs.push_back(maxLineLoc);
    }

    //合并
    if (group && maxLineScores.size()>1)
    {
        //停止标志位
        bool stop = false;
        while (!stop)
        {
            float minDist = groupThreshold + 5;
            float dist = 0.;
            vector<cv::Point>::iterator iloc, jloc, minIloc = maxLineLocs.begin(), minJloc=minIloc+1;
            vector<double>::iterator iscore, jscore, minIscore, minJscore;
            //找出两条线距离的最小值//

            for (iloc=maxLineLocs.begin(), iscore=maxLineScores.begin(); iloc!=maxLineLocs.end(); iloc++, iscore++)
            {
                for (jscore=iscore+1, jloc=iloc+1; jscore!=maxLineScores.end(); jloc++, jscore++)
                {
                    // float t1 = thetas[iloc->x]<0 ? thetas[iloc->x] : thetas[iloc->x]+CV_PI;
                    // float t2 = thetas[jloc->x]<0 ? thetas[jloc->x] : thetas[jloc->x]+CV_PI;
                    //计算距离
                    dist = fabs(rs[iloc->y]-rs[jloc->y]) + 0.1 * fabs(thetas[iloc->x]-thetas[jloc->x]);
                    if (dist<minDist)
                    {
                        minDist = dist;
                        minIloc = iloc; 
                        minIscore = iscore;
                        minJloc = jloc; 
                        minJscore = jscore;
                    }
                }
            }
            
            //如果最小值大于阈值，则停止合并//
            if (minDist >= groupThreshold)
            {
                stop = true;
            }
            else
            {
                //合并线条//
                double x =  (minIloc->x * *minIscore + minJloc->x * *minJscore) / (*minIscore + *minJscore);
                double y =  (minIloc->y * *minIscore + minJloc->y * *minJscore) / (*minIscore + *minJscore);

                minIloc->x = (int)x;// ((minJloc->x + minJloc->x)/2.0); // (int) x;
                minIloc->y = (int)y;// ((minJloc->y + minIloc->y)/2.0); // (int) y;
                *minIscore = (*minJscore + *minIscore)/2;///2;
                maxLineLocs.erase(minJloc);
                maxLineScores.erase(minJscore);

                // 交换顺序//               
                // for (iscore=maxLineScores.begin(), iloc=maxLineLocs.begin(); iscore!=maxLineScores.end() && *minIscore <= *iscore; iscore++, iloc++);

                // if (iscore!=minIscore )
                // {
                //     std::cout<<"erase one"<<std::endl;
                    
                //     //insert in new position
                //     maxLineScores.insert(iscore, *minIscore);
                //     maxLineLocs.insert(iloc, *minIloc);
                //     //delte old
                //     maxLineScores.erase(minIscore);
                //     maxLineLocs.erase(minIloc);
                // }
            }
        }
    }


    //直线坐标转换，得到线段端点
    for(int i=0; i<int(maxLineScores.size()); i++)
    {
        if (maxLineScores[i]>=detectionThreshold)
        {
            YatLine line;
            assert(maxLineLocs[i].x>=0 && maxLineLocs[i].x<thetaBins);
            assert(maxLineLocs[i].y>=0 && maxLineLocs[i].y<rBins);
            yatIntersectLineRThetaWithBB(rs[maxLineLocs[i].y], thetas[maxLineLocs[i].x], cv::Size(image.cols, image.rows), &line);

            std::cout<<"row is: "<< rs[maxLineLocs[i].y]<<"  theta is: "<<thetas[maxLineLocs[i].x] << std::endl;
            lines->push_back(line);
            if (lineScores)
            {
                (*lineScores).push_back(maxLineScores[i]);
            }
        }
        else 
        {
            break;
        }
    }

    //清理//
    delete [] rs;
    delete [] thetas;
    maxLineScores.clear();
    maxLineLocs.clear();
}

/***********************************************************
函数名称：yatSetParam
函数功能：参数初始化
入口参数：无
出口参数：无
备 注：
***********************************************************/
void YatProcRgbLine::yatSetParam()
{
    yatGetMap(mapx, mapy);
}

/***********************************************************
函数名称：yatGetLines
函数功能：获取图像上的直线
入口参数：srcImage:输入灰度图
出口参数：inlines:提取到的直线
备 注：
***********************************************************/
int YatProcRgbLine::yatGetLines(cv::Mat srcImage, std::vector<YatLine> &inlines)
{
    cv::Mat filteredImage, threshedImage, ipmImage, normImage;
    vector<float> lineScores;  
    std::cout<<srcImage.size()<<std::endl;
    
    //转换到 0～1//
    srcImage.convertTo(normImage, CV_32FC1);
    normImage = normImage / 255.0;
    std::cout<<normImage.size()<<std::endl;
    cv::imshow("normimg", normImage);
    cv::waitKey(0);

    //转换成俯视图//
    cv::remap(normImage, ipmImage, mapx, mapy, CV_INTER_LINEAR);
    

    //用特定巻积核处理图像//
    yatFilterLines(ipmImage, filteredImage);

    //把负数响应值置0//
    cv::threshold(filteredImage, threshedImage, 0, 0, cv::THRESH_TOZERO);
    
    //std::cout<<threshedImage<<std::endl;
    //cv::imshow("filteredImage", threshedImage * 50);

    //获取前98%的响应区域的值//
    float qtileThreshold = yatGetQuantile(threshedImage, 0.98);

    //std::cout<<"quant is "<<qtileThreshold<<std::endl;
    
    //二值分割//
    cv::threshold(threshedImage, threshedImage, qtileThreshold, 255, cv::THRESH_BINARY);
    cv::imshow("threshedImage", threshedImage);

    //用改进的霍夫变换检测直线//
	yatGetHoughTransformLines(threshedImage, &inlines,  &lineScores);

    // for(int i=0; i<inlines.size(); i++)
    // {
    //     std::cout<< inlines[i].startPoint<<std::endl;
    //     std::cout<< inlines[i].endPoint<<std::endl;

    //     cv::line(ipmImage, inlines[i].startPoint, inlines[i].endPoint, cv::Scalar(0, 100, 100), 1);
    // }

    // cv::imshow("ipmImg", ipmImage);
    // cv::waitKey(0);
}

/***********************************************************
函数名称：yatPerspectiveToMaps
函数功能：获取原图与转换后的图的映射关系，
入口参数：perspective_mat：透视变换矩阵
        width，height:透视变换原始图像的宽和高
出口参数：mapx:存放x坐标的映射
        mapy:存放y坐标的映射
备 注：
***********************************************************/
void YatProcRgbLine::yatPerspectiveToMaps(cv::Mat perspective_mat, int width , int height, cv::Mat &mapsx, cv::Mat &mapsy)
{
    cv::Mat xy = cv::Mat(height, width, CV_32FC2);
    cv::Mat xy_transformed;
    cv::Mat maps[3];
    for (int y = 0; y < height; y++)
    {
        float *pxy = xy.ptr<float>(y); 
  	    for (int x = 0; x < width; x++)
        {
            *pxy++ = x;
            *pxy++ = y;
        }
    }

    // 透视变换
    cv::warpPerspective(xy, xy_transformed, perspective_mat,  xy.size(), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
    cv::split(xy_transformed, maps);
    mapsx = maps[0];
    mapsy = maps[1];
}

/***********************************************************
函数名称：yatGetMap
函数功能：获取原图与转换后的图的映射关系，
入口参数：无
出口参数：mapx:存放x坐标的映射
        mapy:存放y坐标的映射
返回：   成功返回０，错误返回－１
备 注：
***********************************************************/
int YatProcRgbLine::yatGetMap(cv::Mat &mapx, cv::Mat &mapy)
{
    cv::Point2f srcTri[4], dstTri[4];

    //1.8mm Lens  640x480//
	srcTri[0].x = 260;
	srcTri[0].y = 200;
	srcTri[1].x = 380;
	srcTri[1].y = 200;
	srcTri[2].x = 639;
	srcTri[2].y = 479;
	srcTri[3].x = 0;
	srcTri[3].y = 479;
	
	dstTri[0].x = 80;
	dstTri[0].y = 0;
	dstTri[1].x = 560;
	dstTri[1].y = 0;
	dstTri[2].x = 560;
	dstTri[2].y = 480;
	dstTri[3].x = 80;
	dstTri[3].y = 480;

    //获得透视变换矩阵//
	cv::Mat warp_mat = cv::getPerspectiveTransform(srcTri, dstTri);
  	yatPerspectiveToMaps(warp_mat, IMG_WIDTH, IMG_HEIGHT, mapx, mapy);
    return 0;
}