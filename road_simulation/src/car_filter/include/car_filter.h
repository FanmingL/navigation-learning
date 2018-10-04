/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_filter.h
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:51
*   Description : 1,
*
*===============================================================*/


#ifndef _CAR_FILTER_H
#define _CAR_FILTER_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/tracking.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <unordered_set>
#include <mutex>
#include <algorithm>
#include <thread>
#include <unordered_map>
#include "filter_algorithm.h"

class car_filter {
public:
    class car_data {
        friend std::ostream &operator<<(std::ostream &out, car_data &object);

    public:
        car_data();

        ~car_data() = default;

        int car_index;
        cv::Rect2d bbox;
        int it_frame_index;
        static int frame_index;
        bool found_flag;
    };

    class single_car_filter {
    public:
        single_car_filter();

        ~single_car_filter() = default;

        int start_frame, end_frame;

    };

    car_filter(double overlap_threshold = 0.7);

    car_filter(const std::string &_out_data_path, double overlap_threshold = 0.7);

    car_filter(const std::string &_in_data_path, const std::string &_no);

    ~car_filter();

    bool read_one_car(std::ifstream &_ff, car_data &_one_car);

    bool read_one_car();

    /*only txt*/
    bool read_one_frame_data();

    bool read_next_frame(std::vector<car_data> &_frame_data, cv::Mat &img);

    bool read_next_frame();

    bool run(cv::Mat &dst, std::vector<std::pair<int, cv::Rect2d> > &res);

    void sort_car_by_index(std::vector<car_data> &_car_vector);

    void sort_car_by_index();

    const std::string base_path = std::string(PATH);

    static double calculate_overlap(cv::Rect2d &rect1, cv::Rect2d &rect2);

    int frame_width, frame_height;
    std::vector<car_data> one_frame_car;
private:
    std::ifstream ff;
    std::ofstream of;
    cv::VideoCapture videoCapture;
    car_data one_car;
    cv::Mat one_frame_image;
    std::unordered_map<int, std::vector<cv::Rect2d> > car_buffer;
    std::unordered_map<int, std::shared_ptr<filter_algorithm_base> > car_filter_buffer;
    int index_max_now;
    bool if_write_data;
    double overlap_threshold;
};

#endif //CAR_FILTER_H
