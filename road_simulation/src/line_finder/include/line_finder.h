/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : line_finder.h
*   Author      : FanmingL
*   Created date: 2018-09-30 15:50:13
*   Description : 
*
*===============================================================*/


#ifndef _LINE_FINDER_H
#define _LINE_FINDER_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/calib3d.hpp"
#include "car_filter.h"
#include <queue>

class line_finder{
public:
    line_finder();
    bool run(cv::Mat &dst);
    void find_line(cv::Mat &src, cv::Mat &dst, std::vector<std::pair<cv::Point2d, cv::Point2d> > &res );
    ~line_finder() = default;

    cv::VideoCapture videoCapture;
    const std::string base_path = std::string(PATH);
    cv::Mat canvas, image_now, canvas_src, canvas_dst;
    cv::Rect roi_src, roi_processed;
    static cv::Rect make_rect_from_point(const cv::Point &tl, const cv::Point &br);
    car_filter car_filter1;
    std::vector<car_filter::car_data> car_data;
    bool bfs(const cv::Point &p1, cv::Point &dst, const cv::Mat &img_roi, const std::unordered_set<int> &p_set, cv::Mat &label);
    void filter_line(const std::vector<std::pair<cv::Point, cv::Point> > &src,
            std::vector<std::pair<cv::Point, cv::Point> > &dst,
            std::vector<int> &_index_res);
    class classifier{
    public:
        classifier(const cv::Rect &_bbox, int _index_it);
        int index_it;
        cv::Point p1_last, p2_last;
        double length, angel;
        bool init_flag;
        cv::Rect bbox;
        static int index;
    };
private:
    cv::Mat homography_matrix;
    std::vector<cv::Point2f> world_position;
    std::vector<cv::Point> four_direction;
    std::ofstream of;
    std::vector<line_finder::classifier> line_list;
    std::unordered_set<int> center_point_set;
    std::vector<cv::Point> center_point_vector;
};

#endif //LINE_FINDER_H
