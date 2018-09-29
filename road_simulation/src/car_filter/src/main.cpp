/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:39
*   Description : 
*
*===============================================================*/


#include "car_filter.h"

int main(int argc, char **argv)
{
    car_filter car_filter1;
    //std::remove((char*)(car_filter1.base_path + "/data/out_video.mp4").c_str());
    cv::Mat dst;
    std::vector<std::pair<int, cv::Rect2d> > res;
    int count = 0;
    while (car_filter1.run(dst, res))
    {
#ifdef SHOW_VIDEO
        cv::imshow("11", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif        
        std::cout<<count++<<std::endl;
    }

    return 0;
}

