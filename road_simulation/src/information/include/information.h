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
    explicit information(float _distance_threshold = 1000.0f);
    template <typename T>
    void read_from_binary(T *proto_data, const std::string &path){
        std::ifstream _ff(path, std::ios::binary);
        proto_data->ParseFromIstream(&_ff);
    }
    template <typename T>
    void write_to_binary(T *proto_data, const std::string &path){
        std::ofstream _ff(path, std::ios::binary | std::ios::trunc);
        proto_data->SerializePartialToOstream(&_ff);
    }
    void run();
    rs::video video_data;
    const std::string base_path = PATH;
    rs::all_trajectory all_trajectory;
private:
    float distance_threshold;

};


#endif //INFORMATION_H
