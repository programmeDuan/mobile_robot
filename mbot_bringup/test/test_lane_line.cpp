/***********************************************************
文件名称：get_lane_line
作 者： 王斌
版 本：　０.0.1
说 明：从ＲＧＢ图像中提取车道线
修改记录：
***********************************************************/

#include "rgb_line.hpp"


// main //
int main(int argc, char** argv)
{
    YatProcRgbLine procline;

    procline.yatSetParam();

    cv::Mat clrImage, bImage, f32Image, normImage, ipmImage;
    cv::Mat channels[3];
    std::vector<YatLine> lines;

    //读入图像//
    bImage = cv::imread(FILENAME, 0);
    cv::imshow("srcimg", bImage);
    
    //获取线段//
    procline.yatGetLines(bImage, lines);
}