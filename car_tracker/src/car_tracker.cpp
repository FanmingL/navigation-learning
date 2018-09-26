/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_tracker.cpp
*   Author      : FanmingL
*   Created date: 2018-09-26 22:13:36
*   Description : 
*
*===============================================================*/


#include "car_tracker.h"

car_tracker::car_tracker(float _position_cov, float _control_cov, int _max_count, double _min_area,
                         double _min_over_lap, double _probility_threshold):base_path(PATH){
    current_frame = 0;
    std::unordered_set<std::string> string_may_be;
    string_may_be.insert("car");
    string_may_be.insert("truck");
    string_may_be.insert("phone");
    string_may_be.insert("remote");

    yolo_car::set_parameter(_min_area, string_may_be, _probility_threshold);
    single_tracker::set_parameter(_position_cov, _control_cov, _min_over_lap, _max_count);
    videoCapture.open(base_path+"/data/in_video.mp4");
    assert(videoCapture.isOpened());
    ff.open(base_path+"/data/in_data.txt");
    assert(ff.is_open());

    read_one_car(ff, one_car);
    frame_width = (int)videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
    frame_height = (int)videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
    frame_count = (int)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
    assert(frame_height == frame_width);
}

double car_tracker::yolo_car::min_area = 0;
std::unordered_set<std::string> car_tracker::yolo_car::name_may_be = std::unordered_set<std::string>();
double car_tracker::yolo_car::probility_threshold = 0;
MATRIX_TYPE car_tracker::single_tracker::position_cov = MATRIX_I;
MATRIX_TYPE car_tracker::single_tracker::control_cov = MATRIX_I;
double car_tracker::single_tracker::min_overlap = 0;
int car_tracker::single_tracker::max_count = 0;
int car_tracker::single_tracker::index = 0;
int car_tracker::yolo_car::frame_index = 0;

void car_tracker::yolo_car::set_parameter(double &_min_area, std::unordered_set<std::string> &potential_name,
                                           double _probility_threshold) {
    min_area = _min_area;
    name_may_be = potential_name;
    probility_threshold = _probility_threshold;

}

void car_tracker::single_tracker::set_parameter(float _position_cov, float _control_cov, double _min_overlap,
                                                  int _max_count) {
    position_cov = MATRIX_I * _position_cov;
    control_cov = MATRIX_I * _control_cov;
    min_overlap = _min_overlap;
    max_count = _max_count;
}


bool car_tracker::read_one_car(std::ifstream &_ff, car_tracker::yolo_car &one_car) {
    std::string tmp;
    if (std::getline(ff, tmp))
    {
        std::stringstream ss(tmp);
        ss>>one_car.frame_index>>one_car.name;
        if (one_car.name == "cell")
            ss>>one_car.name;
        ss>>one_car.bbox.x\
        >>one_car.bbox.y>>one_car.bbox.width>>one_car.bbox.height>>one_car.probility;
        one_car.bbox.x -= one_car.bbox.width/2.0;
        one_car.bbox.y -= one_car.bbox.height/2.0;
        one_car.bbox.x *= frame_width;
        one_car.bbox.y *= frame_height;
        one_car.bbox.width *= frame_width;
        one_car.bbox.height *= frame_height;
        one_car.find_couple = false;
        return true;
    }
    return false;
}

void car_tracker::yolo_car::set_current_frame_index(int _current_frame_index) {
    frame_index = _current_frame_index;
}

car_tracker::yolo_car::yolo_car() {

}