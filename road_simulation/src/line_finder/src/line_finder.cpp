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

    std::vector<cv::Rect> roi_buffer;
    roi_buffer.push_back(make_rect_from_point(cv::Point(418, 364), cv::Point(603, 618)));

    cv::Mat gray, corner_image, corner_norm_image, corner_scaled_image;
    cv::Mat roi_image = src(roi_buffer.front());
    cv::cvtColor(roi_image, gray, CV_BGR2GRAY);
    cv::cornerHarris(gray, corner_image, 2, 3, 0.04);
    //std::cout<<corner_image<<std::endl;
    cv::normalize(corner_image, corner_norm_image, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    cv::convertScaleAbs(corner_norm_image, corner_scaled_image);
    dst = src.clone();

    for( int i = 0; i < corner_norm_image.rows ; i++ )
    {
        for( int j = 0; j < corner_norm_image.cols; j++ )
        {
            if(  corner_image.at<float>(i,j) > 0.001 )
            {
                circle( dst, cv::Point(j + roi_buffer.front().tl().x, i + roi_buffer.front().tl().y), 5,  cv::Scalar(0, 0, 255), 2, 8, 0 );
            }
        }
    }



}

cv::Rect line_finder::make_rect_from_point(const cv::Point &tl, const cv::Point &br) {
    cv::Rect res;
    res.x = tl.x;
    res.y = tl.y;
    res.width = br.x - tl.x;
    res.height = br.y - tl.y;
    return res;
}
