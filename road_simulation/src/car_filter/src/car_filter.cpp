 /*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_filter.cpp
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:44
*   Description : 
*
*===============================================================*/


#include "car_filter.h"

int car_filter::car_data::frame_index = 0;

std::ostream &operator<<(std::ostream &out, car_filter::car_data &object) {
    out << object.it_frame_index << " " << object.car_index << " " << object.bbox;
    return out;
}

car_filter::car_data::car_data() {
    found_flag = false;
}

car_filter::car_filter(double _overlap_threshold) {
    /*readonly*/
    ff.open(base_path + "/data/tracker_out_data.txt");
    assert(ff.is_open());
    videoCapture.open(base_path + "/data/in_video.mp4");
    assert(videoCapture.isOpened());
    read_one_car();
    index_max_now = 0;
    overlap_threshold = _overlap_threshold;
}

bool car_filter::read_one_car() {
    return read_one_car(ff, one_car);
}

bool car_filter::read_one_car(std::ifstream &_ff, car_filter::car_data &_one_car) {
    std::string tmp;
    if (std::getline(_ff, tmp)) {
        std::stringstream ss(tmp);
        ss >> _one_car.it_frame_index >> _one_car.car_index >> _one_car.bbox.x >>
           _one_car.bbox.y >> _one_car.bbox.width >> _one_car.bbox.height;
        return true;
    }
    return false;
}

bool car_filter::read_next_frame(std::vector<car_filter::car_data> &_frame_data, cv::Mat &img) {
    _frame_data.clear();
    if (car_data::frame_index >= one_car.it_frame_index) return false;
    videoCapture >> img;
    if (img.empty())return false;
    car_data::frame_index++;
    while (car_data::frame_index == one_car.it_frame_index) {
        _frame_data.push_back(one_car);
        if (!read_one_car())break;
    }
    return true;
}

bool car_filter::read_next_frame() {
    return read_next_frame(one_frame_car, one_frame_image);
}

bool car_filter::run(cv::Mat &dst, std::vector<std::pair<int, cv::Rect2d> > &res) {
    res.clear();
    if (!read_next_frame())return false;
    one_frame_image.copyTo(dst);
    std::unordered_set<int> index_accepted_this_frame;
    std::vector<int> index_to_delete;
    int i = 0;
    for (; i < one_frame_car.size(); ++i) {
        if (one_frame_car[i].car_index > index_max_now)
            break;
        if (one_frame_car[i].car_index <= index_max_now && car_buffer.count(one_frame_car[i].car_index)) {
            index_accepted_this_frame.insert(one_frame_car[i].car_index);
            cv::Rect2d tmp_rect;
            car_filter_buffer[one_frame_car[i].car_index]->run(one_frame_car[i].bbox, one_frame_image, tmp_rect);
            car_buffer[one_frame_car[i].car_index].push_back(tmp_rect);
        }
    }

    for (auto &item:car_buffer){
        if (!index_accepted_this_frame.count(item.first)) {
            index_to_delete.push_back(item.first);
        }
    }

    for (auto &item : index_to_delete) {

        car_buffer.erase(item);
        car_filter_buffer.erase(item);
    }

    for (; i < one_frame_car.size(); i++) {
        index_max_now = one_frame_car[i].car_index;
        int flag = true;
        for (auto &item : index_accepted_this_frame) {
            if (calculate_overlap(car_buffer[item].back(), one_frame_car[i].bbox) > overlap_threshold) {
                flag = false;
                break;
            }
        }
        if (flag) {
            index_accepted_this_frame.insert(one_frame_car[i].car_index);
            car_buffer[one_frame_car[i].car_index].push_back(one_frame_car[i].bbox);
            // TODO: keep add low_pass filter and feature extract filter
            car_filter_buffer[one_frame_car[i].car_index] = std::make_shared<move_mean>(one_frame_car[i].bbox, 3);
        }
    }

    for (auto &item :index_accepted_this_frame) {
        res.emplace_back(std::pair<int, cv::Rect2d>(item, car_buffer[item].back()));
        cv::rectangle(dst, car_buffer[item].back(), cv::Scalar(0, 0, 128), 2);
    }

    return true;
}

void car_filter::sort_car_by_index(std::vector<car_filter::car_data> &_car_vector) {
    std::sort(_car_vector.begin(), _car_vector.end(),
              [](car_data &c1, car_data &c2) -> bool { return c1.car_index > c2.car_index; });
}

void car_filter::sort_car_by_index() {
    sort_car_by_index(one_frame_car);
}

double car_filter::calculate_overlap(cv::Rect2d &rect1, cv::Rect2d &rect2) {
    double a1 = rect1.area(), a2 = rect2.area();
    if (a1 == 0 || a2 == 0)return 0;
    double a3 = (rect1 & rect2).area();
    if (a3 == 0)return 0;
    return (a3 / std::min(a1, a2));
}


car_filter::single_car_filter::single_car_filter() {

}
