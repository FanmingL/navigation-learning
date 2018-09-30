//
// Created by Fanming Luo on 2018/9/29.
//

#ifndef CAR_FILTER_FILTER_ALGORITHM_H
#define CAR_FILTER_FILTER_ALGORITHM_H

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

class filter_algorithm_base {
public:
    filter_algorithm_base() = default;

    ~filter_algorithm_base() = default;

    virtual void init(cv::Rect2d &rect, cv::Mat &image) = 0;

    virtual void run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out) = 0;

    void convert(const cv::Point2d &br, const cv::Point2d &tl, cv::Rect2d &out);
};

class low_pass_filter : public filter_algorithm_base {
public:
    explicit low_pass_filter(cv::Rect2d &rect);

    ~low_pass_filter() = default;

    void init(cv::Rect2d &rect, cv::Mat &image);

    void run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out);

private:
};

class move_mean : public filter_algorithm_base {
public:
    explicit move_mean(cv::Rect2d &rect, int _batch_size = 6);

    ~move_mean() = default;

    void init(cv::Rect2d &rect, cv::Mat &image);

    void run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out);

private:
    int batch_size;
    std::vector<cv::Point2d> ptl_v, pbr_v;
    cv::Point2d ptl_sum, pbr_sum;
    int index;
    cv::Rect2d last_rect;
};

class get_feature : public filter_algorithm_base {
public:
    get_feature(cv::Rect2d &rect, cv::Mat &image);

    ~get_feature() = default;

    void init(cv::Rect2d &rect, cv::Mat &image);

    void run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out);


private:

};


#endif //CAR_FILTER_FILTER_ALGORITHM_H
