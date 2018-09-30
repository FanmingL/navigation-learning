/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : line_finder.cpp
*   Author      : FanmingL
*   Created date: 2018-09-30 15:50:02
*   Description : 
*
*===============================================================*/


#include <src/line_finder/include/line_finder.h>

#include "line_finder.h"

line_finder::line_finder():canvas(960, 960 * 2, CV_8UC3, cv::Scalar(0, 0, 0)),
roi_src(0, 0, 960, 960),
roi_processed(960, 0, 960, 960)
{
    videoCapture.open(base_path + "/data/in_video.mp4");
    assert(videoCapture.isOpened());
    canvas_src = cv::Mat(canvas, roi_src);
    canvas_dst = cv::Mat(canvas, roi_processed);

}

bool line_finder::run(cv::Mat &dst) {
    cv::Mat image_res;
    std::vector<std::pair<cv::Point2d, cv::Point2d> > res_line;
    videoCapture >> image_now;
    if (image_now.empty())return false;

    find_line(image_now, image_res, res_line);

    image_now.copyTo(canvas_src);
    image_res.copyTo(canvas_dst);

    canvas.copyTo(dst);
    return true;

}

void line_finder::find_line(cv::Mat &src, cv::Mat &dst, std::vector<std::pair<cv::Point2d, cv::Point2d> > &res) {
    res.clear();
    cv::Mat gray, gray_mask, color_mask;
    std::vector<cv::Mat> bgr_channel;
    cv::split(src, bgr_channel);
    std::pair<int, int> b_range(230,255),
                        g_range(230,255),
                        r_range(230,255);
    color_mask = ((bgr_channel[0] >= b_range.first) & bgr_channel[0] <= b_range.second)
                &((bgr_channel[1] >= g_range.first) & bgr_channel[1] <= g_range.second)
                & ((bgr_channel[2] >= r_range.first) & bgr_channel[2] <= r_range.second);

    cv::Canny(color_mask,color_mask,150, 200);
    auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    //cv::dilate(color_mask, color_mask, element);
    src.copyTo(dst, color_mask);

}
