/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : information.h
*   Author      : FanmingL
*   Created date: 2018-10-05 13:51:48
*   Description : 
*
*===============================================================*/


#ifndef _INFORMATION_H
#define _INFORMATION_H

#include "car_filter.h"
#include "road_car.pb.h"

class information{
public:
    information(float _distance_threshold = 1000.0f);
    void read_from_proto(rs::video *_video_data, const std::string &path);
    void run();
    void calculate_surround(const rs::object &object, const rs::frame &frame, rs::object_pro &object_pro);
    rs::video video_data;
    const std::string base_path = PATH;
    rs::all_trajectory all_trajectory;
private:
    float distance_threshold;

};


#endif //INFORMATION_H
