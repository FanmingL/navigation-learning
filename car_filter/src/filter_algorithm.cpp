//
// Created by Fanming Luo on 2018/9/29.
//

#include "filter_algorithm.h"
/***       low_pass_filter!!!!!!   *****/
void low_pass_filter::init(cv::Rect2d &rect, cv::Mat &image) {

}

void low_pass_filter::run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out) {

}

low_pass_filter::low_pass_filter(cv::Rect2d &rect) {

}
/**      move_mean!!!!!  ***/
void move_mean::init(cv::Rect2d &rect, cv::Mat &image) {

}

void move_mean::run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out) {
    //std::cout<<rect_in<<last_rect<<std::endl;
    if (rect_in.area() == 0)
    {
        rect_out = rect_in;
        return ;
    }
    if ((rect_in & last_rect).area() == 0)
    {
        rect_out = rect_in;
        last_rect = rect_out;
        return ;
    }
    pbr_sum -= pbr_v[index];
    ptl_sum -= ptl_v[index];

    pbr_v[index] = rect_in.br();
    ptl_v[index] = rect_in.tl();

    pbr_sum += pbr_v[index];
    ptl_sum += ptl_v[index];
    if((++index) == batch_size)index = 0;
    convert(pbr_sum / batch_size, ptl_sum / batch_size, rect_out);
    last_rect = rect_out;
    //rect_out = rect_in;

}

move_mean::move_mean(cv::Rect2d &rect, int _batch_size) :batch_size(_batch_size),
index(0)
{
    last_rect = rect;
    for (int i = 0; i < _batch_size; ++i)
    {
       pbr_v.push_back(rect.br());
       ptl_v.push_back(rect.tl());
       pbr_sum += rect.br();
       ptl_sum += rect.tl();
    }

}

/**     get_feature!!!!     ****/
void get_feature::init(cv::Rect2d &rect, cv::Mat &image) {

}

void get_feature::run(cv::Rect2d &rect_in, cv::Mat &image, cv::Rect2d &rect_out) {

}

get_feature::get_feature(cv::Rect2d &rect, cv::Mat &image) {

}

void filter_algorithm_base::convert(const cv::Point2d &br, const cv::Point2d &tl, cv::Rect2d &out) {
    out.x = tl.x;
    out.y = tl.y;
    out.width = br.x - tl.x;
    out.height = br.y - tl.y;
}
