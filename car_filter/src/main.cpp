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
    cv::VideoCapture cap(car_filter1.base_path + "/data/in_video.mp4");
    assert(cap.isOpened());
    std::remove((char*)(car_filter1.base_path + "/data/out_video.mp4").c_str());
    

    return 0;
}

