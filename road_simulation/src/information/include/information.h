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
    enum ROAD_TYPE{
        ROAD_CANOT_REACH = 0,
        LEFT_BIG_ROAD,
        MIDDLE_ROAD_INDEX_1,
        MIDDLE_ROAD_INDEX_2,
        MIDDLE_ROAD_INDEX_3,
        MIDDLE_LEFT_APPEND,
        CAN_REACH_SURROUND_WITH_LINE,
        RIGHT_DOWN_BIG_ROAD,
        RIGHT_ROAD_INDEX_1,
        RIGHT_ROAD_INDEX_2
    };
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
    rs::all_homograph_matrix all_matrix;
    rs::all_trajectory all_trajectory;
    void print_one_trajectory(const rs::trajectory &trajectory);
private:
    float distance_threshold;

};


#endif //INFORMATION_H
