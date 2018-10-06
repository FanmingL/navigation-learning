/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : relocation.h
*   Author      : FanmingL
*   Created date: 2018-10-04 18:49:21
*   Description : 
*
*===============================================================*/


#ifndef _RELOCATION_H
#define _RELOCATION_H

#include "car_filter.h"
#include "opencv2/calib3d.hpp"
#include "road_car.pb.h"
#include <iomanip>

class relocation {
public:
    class car_data {
    public:
        car_data(const int &_index, const int &_frame_count, const cv::Rect2d &world, const cv::Rect2d &image, const cv::Point2d &_world_center) :
                index(_index), frame_count(_frame_count), bbox_in_world(world), bbox_in_image(image),center_in_world(_world_center) {}

        int index;
        int frame_count;
        cv::Rect2d bbox_in_world;
        cv::Rect2d bbox_in_image;
        cv::Point2d center_in_world;
    };

    relocation();

    bool read_one_frame(cv::Mat &img, std::vector<car_filter::car_data> &car_data, cv::Mat &homography);

    void get_first_homography(cv::Mat &img, cv::Mat &_homography_base);

    bool run(cv::Mat &dst, std::vector<car_data> &res);

    template<typename T>
    std::string to_string_with_precision(const T a_value, const int n = 5) {
        std::ostringstream out;
        out << std::setprecision(n) << a_value;
        return out.str();
    }

    car_filter car_reader;
    cv::Mat homography_to_first;
private:
    cv::Point2d original_point;
    cv::Mat homography_base;
    std::ifstream ff;
    cv::Mat current_image;
    std::vector<car_filter::car_data> current_frame_car;

};

#endif //RELOCATION_H
