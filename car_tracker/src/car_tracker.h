/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_tracker.h
*   Author      : FanmingL
*   Created date: 2018-09-26 22:13:42
*   Description : 
*
*===============================================================*/


/*
 *
 *
 *
 *
 *
 */
#ifndef _CAR_TRACKER_H
#define _CAR_TRACKER_H

#include "KalmanFilter.h"
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

class car_tracker{
public:
    class yolo_car{
    public:
        yolo_car();
        ~yolo_car() = default;
        bool pre_filter();
        static void set_parameter(double &_min_area,
                std::unordered_set<std::string> &_potential_name, double _probility_threshold);
        static void set_current_frame_index(int _current_frame_index);
        cv::Rect2d bbox;
        std::string name;
        double probility;
        bool find_couple;
        /*car truck phone remote*/
        static std::unordered_set<std::string> name_may_be;
        static double min_area, probility_threshold;
        static int frame_index;
    };
    class single_tracker{
    public:
        /*    method   */
        single_tracker(cv::Mat &image, cv::Rect2d &rect);
        ~single_tracker() = default;
        void init(cv::Mat &image, cv::Rect2d &rect);
        bool run(cv::Mat &image, std::vector<car_tracker::yolo_car> &car_current_frame, cv::Rect2d &res);
        static double calculate_overlap(cv::Rect2d &rect1, cv::Rect2d &rect2);
        static void set_parameter(float _position_cov, float _control_cov, double _min_overlap, int _max_count);
        /*   variable */
        int count_now;
        /* LF RF RB LB */
        std::vector<KalmanFilter> kalman_filter_set;
        cv::Ptr<cv::Tracker> ptr;
        static int index, max_count;
        static double min_overlap;
        static MATRIX_TYPE position_cov, control_cov;
    };



    car_tracker(float _position_cov = 1.0f, float _control_cov = 1.0f,
            int _max_count=30, double _min_area=500., double _min_over_lap = 0.4, double _probility_threshold=0.4);
    ~car_tracker()= default;
    bool read_one_car(std::ifstream &_ff, car_tracker::yolo_car &one_car);
    bool run(cv::Mat &dst);
    std::string base_path;
    int frame_width, frame_height, frame_count;
    int current_frame;

private:
    std::vector<car_tracker::yolo_car> objects_current_frame;
    car_tracker::yolo_car one_car;
    cv::VideoCapture videoCapture;
    std::list<car_tracker::single_tracker> algorithms;
    std::ifstream ff;
};

#endif //CAR_TRACKER_H
