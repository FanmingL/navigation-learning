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
private:

};

#endif //LINE_FINDER_H
