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
#include <queue>

#include "line_finder.h"

line_finder::line_finder():canvas(960, 960 * 2, CV_8UC3, cv::Scalar(0, 0, 0)),
roi_src(0, 0, 960, 960),
roi_processed(960, 0, 960, 960)
{
    // videoCapture.open(base_path + "/data/in_video.mp4");
    assert(videoCapture.isOpened());
    canvas_src = cv::Mat(canvas, roi_src);
    canvas_dst = cv::Mat(canvas, roi_processed);
    four_direction.emplace_back(cv::Point(-1, 0));
    four_direction.emplace_back(cv::Point(1, 0));
    four_direction.emplace_back((cv::Point(0, 1)));
    four_direction.emplace_back((cv::Point(0, -1)));
    line_list.emplace_back(line_finder::classifier(cv::Rect(1428 - 960, 480, 1585-1428, 603-480), 0));
    line_list.emplace_back(line_finder::classifier(cv::Rect(1436 - 960, 397, 1575-1436, 487-397), 1));
}

int line_finder::classifier::index = 0;

line_finder::classifier::classifier(const cv::Rect &_bbox, int _index_it) {
    init_flag = false;
    bbox = _bbox;
    index_it = _index_it;
}

bool line_finder::run(cv::Mat &dst) {
    cv::Mat image_res;
    std::vector<std::pair<cv::Point2d, cv::Point2d> > res_line;
    if (!car_filter1.read_next_frame(car_data, image_now))return false;
    find_line(image_now, image_res, res_line);
    image_now.copyTo(canvas_src);
    image_res.copyTo(canvas_dst);

    canvas.copyTo(dst);
    return true;

}


void line_finder::find_line(cv::Mat &src, cv::Mat &dst, std::vector<std::pair<cv::Point2d, cv::Point2d> > &res) {
    res.clear();
    std::vector<cv::Rect> roi_buffer;//364
    cv::Rect ROI= make_rect_from_point(cv::Point(418, 284), cv::Point(603, 618));
    roi_buffer.push_back(ROI);
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    std::vector<cv::Point> interest_points;
    cv::Mat mask = gray > 220;
    auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(mask,mask, element);
    src.copyTo(dst);
    cv::goodFeaturesToTrack(gray(ROI), interest_points, 50, 0.05, 5, cv::Mat(),3,true,0.04);
    std::unordered_set<int> point_may_be, pair_buffer;
    std::list<cv::Point> point_may_be_v;
    for (auto &item : interest_points)
    {
        bool flag = true;
        for (auto &item2 : car_data)
        {
            auto bbox_tmp = item2.bbox;
            bbox_tmp.width += 5;
            bbox_tmp.height += 5;
            bbox_tmp.x -= 2;
            bbox_tmp.y -= 2;
            if (bbox_tmp.contains(cv::Point2d(item.x +ROI.tl().x, item.y + ROI.tl().y)))
            {
                flag = false;
                break;
            }
        }
        if (flag && mask.at<unsigned char>(item+ROI.tl()) == 255)
        {
            point_may_be.insert(item.y * ROI.width + item.x);
            point_may_be_v.push_back(item);
        }
    }

    cv::Mat label(ROI.size(),CV_8UC1,cv::Scalar(0));
    std::vector<std::pair<cv::Point, cv::Point> > line_may_be;
    for (auto iter = point_may_be_v.begin(); iter != point_may_be_v.end(); )
    {
        cv::Point res_point;
        if (pair_buffer.count(iter->x +iter->y *ROI.width)){
            iter++;
            continue;
        }
        if (bfs(*iter, res_point, mask(ROI), point_may_be, label))
        {
            if (std::abs(iter->x - res_point.x) < 10&&\
                std::abs(iter->y - res_point.y) < 70 &&\
                std::abs(iter->y - res_point.y) > 13)
                {
                pair_buffer.insert(iter->x + iter->y *ROI.width);
                pair_buffer.insert(res_point.x + res_point.y * ROI.width);
                if (iter->y > res_point.y)
                    line_may_be.emplace_back(std::pair<cv::Point, cv::Point>(*iter+ROI.tl(), res_point+ROI.tl()));
                else
                    line_may_be.emplace_back(std::pair<cv::Point, cv::Point>(res_point+ROI.tl(), *iter+ROI.tl()));

                //cv::line(dst, *iter + ROI.tl(), res_point +ROI.tl(), cv::Scalar(255, 0, 255), 3);
                iter++;
            } else{
                iter = point_may_be_v.erase(iter);
            }
        }
        else
        {
            iter = point_may_be_v.erase(iter);
        }
    }
    auto line_compare = [](std::pair<cv::Point, cv::Point> &l1, std::pair<cv::Point, cv::Point> &l2)->bool{
        return l1.first.y > l2.first.y;
    };
    std::sort(line_may_be.begin(), line_may_be.end(), line_compare);
    std::vector<int> index_res;
    filter_line(line_may_be, line_may_be, index_res);

    for (auto &item : car_data)
    {
        cv::rectangle(dst, item.bbox, cv::Scalar(128, 0, 0), 2);
    }

    for (int i =0 ;i < line_may_be.size(); i++)
    {
        auto item = line_may_be[i];
        auto p_tmp = (item.first +item.second)/2;
        int center_index_tmp = p_tmp.x + p_tmp.y * car_filter1.frame_width;
        cv::line(dst, item.first, item.second, cv::Scalar(255,0,255), 3);
        cv::putText(dst,std::to_string(index_res[i]), item.first, cv::FONT_ITALIC, 0.5, cv::Scalar(0, 0, 0), 2);
    }
    std::unordered_set<int> index_res_set;
    for(auto &item:index_res)
        index_res_set.insert(item);
    if (index_res_set.empty() || index_res_set.size() == 1 ||
            (
                    index_res_set.size() == 2&&
                            (
                                    (
                                            index_res_set.count(0)&&index_res_set.count(2)
                                            )||
                                            (
                                             index_res_set.count(1)&&index_res_set.count(3)
                                                    )
                                    )
            )
                    )

        std::cout<<"0"<<std::endl;
    else
        std::cout<<"1"<<std::endl;
    //std::cout<<line_list.size()<<std::endl;
}

cv::Rect line_finder::make_rect_from_point(const cv::Point &tl, const cv::Point &br) {
    cv::Rect res;
    res.x = tl.x;
    res.y = tl.y;
    res.width = br.x - tl.x;
    res.height = br.y - tl.y;
    return res;
}

bool line_finder::bfs(const cv::Point &p1, cv::Point &dst, const cv::Mat &img_roi,const  std::unordered_set<int> &p_set, cv::Mat &label) {
    std::queue<cv::Point> q;
    label.at<unsigned char>(p1) = 255;
    if (img_roi.at<unsigned char>(p1) == 0)return false;
    q.push(p1);
    auto sum = q.size();
    int step=  0;

    while (!q.empty()){
        for (int i = 0; i <sum; i++)
        {
            auto p2 = q.front();
            q.pop();
            if (p2 != p1 && p_set.count(p2.x +p2.y * img_roi.cols)){
                dst = p2;
                return true;
            }
            for (auto &item : four_direction)
            {
                auto p_tmp = item + p2;
                if (p_tmp.x >=0 && p_tmp.x < img_roi.cols && p_tmp.y >=0 && p_tmp.y < img_roi.rows)
                {
                    if (label.at<unsigned char>(p_tmp) == 0 && img_roi.at<unsigned char>(p_tmp) > 0)
                    {
                        q.push(p_tmp);
                        label.at<unsigned char>(p_tmp)=255;
                    }
                }
            }
        }
        sum = q.size();
        step++;
    }
    return false;
}

void line_finder::filter_line(const std::vector<std::pair<cv::Point, cv::Point> > &src,
                              std::vector<std::pair<cv::Point, cv::Point> > &dst,
                              std::vector<int> &_index_res) {
    std::vector<std::vector<int> > index_buffer(line_list.size());
    _index_res.clear();
    std::vector<std::pair<cv::Point, cv::Point> > dst_tmp;
    double length_threshold = 1.14;
    double bias_max = 30;
    for(int i = 0 ; i < line_list.size(); i++)
    {
        for (int j = 0 ; j <src.size(); j++)
        {
            auto p = (src[j].first + src[j].second) / 2;
            if (line_list[i].bbox.contains(p))
            {
                index_buffer[i].push_back(j);
            }
        }
    }
    for (int i = 0; i < line_list.size();i ++)
    {
        if ((!line_list[i].init_flag) && index_buffer[i].size() == 2)
        {
            auto p1 = (src[index_buffer[i][0]].first + src[index_buffer[i][0]].second)/2;
            auto p2 = (src[index_buffer[i][1]].first + src[index_buffer[i][1]].second)/2;
            double l1 = cv::norm(src[index_buffer[i][0]].first - src[index_buffer[i][0]].second);
            double l2 = cv::norm(src[index_buffer[i][1]].first - src[index_buffer[i][1]].second);
            if ((l1 / l2) < length_threshold && (l2 / l1) < length_threshold) {
                line_list[i].init_flag = true;
                line_list[i].length = (l1 + l2) / 2;
                if (p1.x < p2.x) {
                    line_list[i].p1_last = p1;
                    line_list[i].p2_last = p2;
                    dst_tmp.push_back(src[index_buffer[i][0]]);
                    dst_tmp.push_back(src[index_buffer[i][1]]);
                } else {
                    line_list[i].p1_last = p2;
                    line_list[i].p2_last = p1;
                    dst_tmp.push_back(src[index_buffer[i][1]]);
                    dst_tmp.push_back(src[index_buffer[i][0]]);
                }
                _index_res.push_back(i * 2);
                _index_res.push_back(i * 2 + 1);
            }
        }
        if (line_list[i].init_flag)
        {
            for (int j = 0; j < index_buffer[i].size();j++)
            {
                auto p = (src[index_buffer[i][j]].first + src[index_buffer[i][j]].second) / 2;

                double length = cv::norm(src[index_buffer[i][j]].first - src[index_buffer[i][j]].second);
                double length_left = cv::norm(p - line_list[i].p1_last),
                        length_right = cv::norm(p - line_list[i].p2_last);
                if (length / line_list[i].length < length_threshold &&
                    line_list[i].length / length < length_threshold &&
                    std::min(length_left, length_right) < bias_max) {
                    dst_tmp.push_back(src[index_buffer[i][j]]);
                    line_list[i].length = length;
                    if (length_left < length_right) {
                        _index_res.push_back(i * 2);
                        line_list[i].p1_last = p;
                    } else {
                        _index_res.push_back(i * 2 + 1);
                        line_list[i].p2_last = p;
                    }
                }
            }
        }
    }
    dst = dst_tmp;
}


