/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-25 15:44:03
*   Description : 
*
*===============================================================*/


#include "tracking.h"

int main(int argc, char **argv)
{
    tracking tracking1;
    cv::Mat dst;
    std::vector<cv::Rect2d> res;
    tracking1.init_trackers();
    cv::VideoWriter videoWriter(tracking1.basePath+"/data/out_video.mp4", CV_FOURCC('D', 'I', 'V', 'X'), 30, cv::Size(960, 960));

#if 1
    while (tracking1.run(dst, res))
    {
        if (dst.empty())continue;
        videoWriter << dst;
    }
#else
    while (tracking1.get_current_frame())
    {
        tracking1.draw_current_frame(dst);
        cv::imshow("1", dst);
        auto key = cv::waitKey(1);
        if (key=='q')break;
    }
#endif
    cv::destroyWindow("1");
    return 0;
}

