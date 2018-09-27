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

car_tracker::car_tracker(float _position_cov, float _control_cov, int _max_count_yolo,
        int _max_count_traker, double _min_area, double _min_over_lap,
        double _probility_threshold):base_path(PATH){
    current_frame = 0;
    std::unordered_set<std::string> string_may_be;
    string_may_be.insert("car");
    string_may_be.insert("truck");
    string_may_be.insert("phone");
    string_may_be.insert("remote");

    yolo_car::set_parameter(_min_area, string_may_be, _probility_threshold);
    single_tracker::set_parameter(_position_cov, _control_cov, _min_over_lap,
            _max_count_yolo, _max_count_traker);

    videoCapture.open(base_path+"/data/in_video.mp4");
    assert(videoCapture.isOpened());
    ff.open(base_path+"/data/in_data.txt");
    assert(ff.is_open());

    frame_width = (int)videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
    frame_height = (int)videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
    frame_count = (int)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);

    assert(frame_height == frame_width);
    read_one_car(ff, one_car);
}

double car_tracker::yolo_car::min_area = 0;
std::unordered_set<std::string> car_tracker::yolo_car::name_may_be = std::unordered_set<std::string>();
double car_tracker::yolo_car::probility_threshold = 0;
MATRIX_TYPE car_tracker::single_tracker::position_cov = MATRIX_I;
MATRIX_TYPE car_tracker::single_tracker::control_cov = MATRIX_I;
double car_tracker::single_tracker::min_overlap = 0;
int car_tracker::single_tracker::max_count_yolo = 0;
int car_tracker::single_tracker::max_count_tracker = 0;
int car_tracker::single_tracker::index = 0;
int car_tracker::yolo_car::frame_index = -1;

void car_tracker::yolo_car::set_parameter(double &_min_area, std::unordered_set<std::string> &potential_name,
                                           double _probility_threshold) {
    min_area = _min_area;
    name_may_be = potential_name;
    probility_threshold = _probility_threshold;

}

void car_tracker::single_tracker::set_parameter(float _position_cov, float _control_cov, double _min_overlap,
                                                  int _max_count_yolo, int _max_count_tracker) {
    position_cov = MATRIX_I * _position_cov;
    control_cov = MATRIX_I * _control_cov;
    min_overlap = _min_overlap;
    max_count_yolo = _max_count_yolo;
    max_count_tracker = _max_count_tracker;
}

car_tracker::single_tracker::single_tracker(cv::Mat &image, cv::Rect2d &rect) {
    ptr = cv::TrackerKCF::create();
    for (int i = 0; i < 2; i++)
    {
        KalmanFilter kalman_filter_tmp(position_cov, control_cov);
        kalman_filter_set.push_back(kalman_filter_tmp);
    }
    count_now_yolo = 0;
    count_now_traker = 0;
    index_it = (index++);
    init(image, rect);
}

void car_tracker::single_tracker::init(cv::Mat &image, cv::Rect2d &rect) {
    init_kf(rect);
    ptr->init(image, rect);
}

double car_tracker::single_tracker::calculate_overlap(cv::Rect2d &rect1, cv::Rect2d &rect2) {
    double a1 = rect1.area(), a2 = rect2.area();
    if (a1==0 || a2 == 0)return 0;
    return ((rect1&rect2).area() / std::min(a1, a2));
}

bool car_tracker::single_tracker::run(cv::Mat &image, std::vector<car_tracker::yolo_car> &car_current_frame,
                                      cv::Rect2d &res) {
    std::vector<std::pair<double, int> > box_satisfied_constraint;
    cv::Rect2d tracker_box_res;
    bool tracker_res = ptr->update(image, tracker_box_res);
    if (tracker_res)feed_dog_tracker();
    for (int i = 0; i < car_current_frame.size();i++) {
        if (car_current_frame[i].check_constraints() && (!car_current_frame[i].find_couple))
        {
            double overlap = calculate_overlap(car_current_frame[i].bbox, tracker_box_res);
            if (overlap > min_overlap)
            {
                box_satisfied_constraint.emplace_back(std::pair<double, int>(overlap, i));
                car_current_frame[i].find_couple = true;
            }
        }
    }

    if (!box_satisfied_constraint.empty())
    {
        feed_dog_yolo();
        std::sort(box_satisfied_constraint.begin(), box_satisfied_constraint.end());
        auto measure = car_current_frame[box_satisfied_constraint.back().second].bbox;
        if (tracker_box_res.width==0 || tracker_box_res.height==0)
        {
            res = measure;
        }
        else
        {
            res = run_kf(measure, tracker_box_res);
            //res = measure;
            //res = tracker_box_res;
        }
        ptr = cv::TrackerKCF::create();
        ptr->init(image, res);
    }
    else
    {
        res = tracker_box_res;
    }
    return !check_dog();
}

bool car_tracker::single_tracker::check_dog() {
    count_now_yolo ++;
    count_now_traker ++;
    return ((count_now_yolo > max_count_yolo\
    || count_now_traker > max_count_tracker));
}

void car_tracker::single_tracker::feed_dog_yolo() {
    count_now_yolo = 0;
}

void car_tracker::single_tracker::feed_dog_tracker(){
    count_now_traker = 0;
}

cv::Rect2d car_tracker::single_tracker::run_kf(cv::Rect2d &measurement, cv::Rect2d &post_estimate) {
    cv::Rect2d res;
    std::vector<VECTOR_TYPE> measure_vector, post_vector;
    VECTOR_TYPE vector_tmp1, vector_tmp2, out;
    /*back right-------top left*/
    cv::Point2d p1 = measurement.br(), p2 = post_estimate.br();
    vector_tmp1(0,0)=p1.x;vector_tmp1(1,0)=p1.y;
    vector_tmp2(0,0)=p2.x;vector_tmp2(1,0)=p2.y;
    kalman_filter_set[0].correct(vector_tmp1, vector_tmp2, out, false);
    cv::Point2d br(out(0,0), out(1,0));
    p1 = measurement.tl(), p2 = post_estimate.tl();
    vector_tmp1(0,0)=p1.x;vector_tmp1(1,0)=p1.y;
    vector_tmp2(0,0)=p2.x;vector_tmp2(1,0)=p2.y;
    kalman_filter_set[1].correct(vector_tmp1, vector_tmp2, out, false);
    cv::Point2d tl(out(0, 0), out(1, 0));
    //std::cout<<p1<<p2<<tl<<std::endl<<kalman_filter_set[1].temp1<<std::endl;

    res.x = tl.x;
    res.y = tl.y;

    res.width = br.x-tl.x;
    res.height = br.y - tl.y;
    return res;
}

void car_tracker::single_tracker::init_kf(cv::Rect2d &measurement) {
    // std::cout<<measurement.br()<<std::endl;
     VECTOR_TYPE vector_tmp1;
    /*back right-------top left*/
    cv::Point2d p1 = measurement.br();
    vector_tmp1(0,0)=p1.x;vector_tmp1(1,0)=p1.y;
    kalman_filter_set[0].setStart(vector_tmp1);
    p1 = measurement.tl();
    vector_tmp1(0,0)=p1.x;vector_tmp1(1,0)=p1.y;
    kalman_filter_set[1].setStart(vector_tmp1);
}

bool car_tracker::read_one_car(std::ifstream &_ff, car_tracker::yolo_car &one_car) {
    std::string tmp;
    if (std::getline(ff, tmp))
    {
        std::stringstream ss(tmp);
        ss>>one_car.index_it>>one_car.name;
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

bool car_tracker::run(cv::Mat &dst, std::vector<cv::Rect2d> &res) {
    if (!read_one_frame(ff, objects_current_frame, image_current_frame))
        return false;
    image_current_frame.copyTo(dst);
    res.clear();
    for (auto &item : objects_current_frame)
    {
        cv::rectangle(dst, item.bbox, cv::Scalar(128, 128, 128), 1);
    }
    for (auto iter = algorithms.begin(); iter != algorithms.end(); )
    {
        cv::Rect2d tmp_res;
        if (iter->run(image_current_frame, objects_current_frame, tmp_res))
        {
            //std::cout<<tmp_res<<std::endl;
            cv::rectangle(dst, tmp_res, cv::Scalar(128, 0, 0), 2);
            cv::putText(dst, std::to_string(iter->index_it),
                    cv::Point(tmp_res.x, tmp_res.y),cv::FONT_HERSHEY_DUPLEX,0.5,
                    cv::Scalar(0, 0, 128));
            res.push_back(tmp_res);
            iter++;
        }
        else
        {
            iter = algorithms.erase(iter);
        }
    }

    std::vector<std::pair<cv::Rect2d, bool> > box_vector;
    for (auto &item : objects_current_frame)
    {
        if ((!item.find_couple) && item.check_constraints())
        {
            box_vector.emplace_back(std::pair<cv::Rect2d, bool>(item.bbox, false));
        }
    }

    for (int i = 0; i < box_vector.size(); i++)
    {
        bool flag = true;
        for (int j = 0; j < i; j++)
        {
            if (box_vector[j].second &&
            single_tracker::calculate_overlap(box_vector[i].first,
                    box_vector[j].first) > single_tracker::min_overlap){
                flag = false;
                break;
            }
        }

        if (flag)
        {
            box_vector[i].second = true;
            algorithms.emplace_back(single_tracker(image_current_frame, box_vector[i].first));
        }
    }

    return true;
}

car_tracker::yolo_car::yolo_car() {

}

bool car_tracker::yolo_car::check_constraints() {
    return ((name_may_be.count(name) &&
    bbox.area() > min_area &&
    probility > probility_threshold));
}

std::ostream & operator << (std::ostream &out,
                            car_tracker::yolo_car &objects) {
    out<<objects.index_it<<", "<<objects.name<<", "
    <<objects.probility<<", "<<objects.bbox;
    return out;
}

bool car_tracker::read_one_frame(std::ifstream &_ff, std::vector<car_tracker::yolo_car> &one_frame_car, cv::Mat &img) {
    one_frame_car.clear();
    if (yolo_car::frame_index >= one_car.index_it) return false;
    videoCapture >> img;
    if (img.empty())return false;
    yolo_car::frame_index ++;
    while (yolo_car::frame_index == one_car.index_it)
    {
        one_frame_car.push_back(one_car);
        if (!read_one_car(ff, one_car))break;
    }
    return true;
}

bool car_tracker::read_one_frame() {
    return read_one_frame(ff, objects_current_frame, image_current_frame);
}
