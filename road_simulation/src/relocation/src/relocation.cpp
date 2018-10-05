/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : relocation.cpp
*   Author      : FanmingL
*   Created date: 2018-10-04 18:49:15
*   Description : 
*
*===============================================================*/


#include <src/relocation/include/relocation.h>

#include "relocation.h"


relocation::relocation() : car_reader("/data/filter_out_data_normal.txt", "no") {
    ff.open(car_reader.base_path + "/data/stabling_matrix.txt");

    read_one_frame(current_image, current_frame_car, homography_base);
    original_point = cv::Point2d(-1000, -1000);
}


bool relocation::read_one_frame(cv::Mat &img, std::vector<car_filter::car_data> &car_data, cv::Mat &homography) {
    if (!car_reader.read_next_frame(car_data, img))return false;
    if (car_filter::car_data::frame_index == 1) {
        get_first_homography(img, homography);
        cv::Mat out_tmp(3, 3, CV_32FC1, cv::Scalar(0));
        out_tmp.at<float>(0, 0) = (float) homography.at<double>(0, 0);
        out_tmp.at<float>(0, 1) = (float) homography.at<double>(0, 1);
        out_tmp.at<float>(0, 2) = (float) homography.at<double>(0, 2);
        out_tmp.at<float>(1, 0) = (float) homography.at<double>(1, 0);
        out_tmp.at<float>(1, 1) = (float) homography.at<double>(1, 1);
        out_tmp.at<float>(1, 2) = (float) homography.at<double>(1, 2);
        out_tmp.at<float>(2, 0) = (float) homography.at<double>(2, 0);
        out_tmp.at<float>(2, 1) = (float) homography.at<double>(2, 1);
        out_tmp.at<float>(2, 2) = (float) homography.at<double>(2, 2);
        homography = out_tmp.clone();
        return true;
    }
    cv::Mat out_tmp(3, 3, CV_32FC1, cv::Scalar(0));
    std::string str_tmp;
    if (!std::getline(ff, str_tmp))return false;
    int current_frame_index;
    std::stringstream ss(str_tmp);
    ss >> current_frame_index >>
       out_tmp.at<float>(0, 0) >> out_tmp.at<float>(0, 1) >> out_tmp.at<float>(0, 2) >>
       out_tmp.at<float>(1, 0) >> out_tmp.at<float>(1, 1) >> out_tmp.at<float>(1, 2) >>
       out_tmp.at<float>(2, 0) >> out_tmp.at<float>(2, 1) >> out_tmp.at<float>(2, 2);
    homography = out_tmp.clone();
    return true;
}

void relocation::get_first_homography(cv::Mat &img, cv::Mat &_homography_base) {
#if 0
    cv::imshow("init", img);
    cv::waitKey(0);
    cv::destroyWindow("init");
#else

    std::vector<cv::Point2d> world_points, image_points;
    world_points.emplace_back(cv::Point2d(0, 0));
    image_points.emplace_back(cv::Point2d(530, 416));

    world_points.emplace_back(cv::Point2d(350, 0));
    image_points.emplace_back(cv::Point2d(579, 427));

    world_points.emplace_back(cv::Point2d(0, 400));
    image_points.emplace_back(cv::Point2d(531, 377));

    world_points.emplace_back(cv::Point2d(350, 400));
    image_points.emplace_back(cv::Point2d(576, 382));

    world_points.emplace_back(cv::Point2d(0, -600));
    image_points.emplace_back(cv::Point2d(527, 490));

    world_points.emplace_back(cv::Point2d(350, -600));
    image_points.emplace_back(cv::Point2d(584, 498));


    world_points.emplace_back(cv::Point2d(0, 1000));
    image_points.emplace_back(cv::Point2d(533, 330));

    world_points.emplace_back(cv::Point2d(350, 1000));
    image_points.emplace_back(cv::Point2d(573, 334));


    _homography_base = cv::findHomography(image_points, world_points);

#endif
}

bool relocation::run(cv::Mat &dst, std::vector<relocation::car_data> &res) {
    res.clear();
    if (!read_one_frame(current_image, current_frame_car, homography_to_first))return false;
    //std::cout<<homography_to_first<<std::endl<<homography_base<<std::endl;
    dst = current_image.clone();
    cv::Mat homo_tmp = homography_base * homography_to_first;
    cv::Mat calcu_tmp(3, 1, CV_32FC1, cv::Scalar(1)), calcu_res;
    for (auto &item : current_frame_car) {
        if (item.bbox.area() == 0)continue;
        cv::rectangle(dst, item.bbox, cv::Scalar(128, 0, 0), 2);
        /// tl
        auto p = item.bbox.tl();
        calcu_tmp.at<float>(0, 0) = (float) p.x;
        calcu_tmp.at<float>(1, 0) = (float) p.y;
        calcu_tmp.at<float>(2, 0) = 1;
        calcu_res = homo_tmp * calcu_tmp;
        calcu_res /= calcu_res.at<float>(2, 0);
        cv::Point2d p_tl(calcu_res.at<float>(0, 0), calcu_res.at<float>(1, 0));
        /// br
        p = item.bbox.br();
        calcu_tmp.at<float>(0, 0) = (float) p.x;
        calcu_tmp.at<float>(1, 0) = (float) p.y;
        calcu_tmp.at<float>(2, 0) = 1;
        calcu_res = homo_tmp * calcu_tmp;
        calcu_res /= calcu_res.at<float>(2, 0);
        cv::Point2d p_br(calcu_res.at<float>(0, 0), calcu_res.at<float>(1, 0));
        p = (item.bbox.br() + item.bbox.tl()) / 2.0;

        calcu_tmp.at<float>(0, 0) = (float) p.x;
        calcu_tmp.at<float>(1, 0) = (float) p.y;
        calcu_tmp.at<float>(2, 0) = 1;
        calcu_res = homo_tmp * calcu_tmp;
        calcu_res /= calcu_res.at<float>(2, 0);
        cv::Point2d p_res(calcu_res.at<float>(0, 0), calcu_res.at<float>(1, 0));
        p_res -= original_point;
        std::string put_to;
        put_to += to_string_with_precision(p_res.x);
        cv::putText(dst, put_to, item.bbox.tl(), cv::FONT_ITALIC, 0.7, cv::Scalar(0, 0, 255), 2);
        put_to = "";
        put_to += to_string_with_precision(p_res.y);
        cv::putText(dst, put_to, item.bbox.tl() + cv::Point2d(0, 20), cv::FONT_ITALIC, 0.7, cv::Scalar(0, 0, 255), 2);
        res.emplace_back(relocation::car_data(item.car_index, car_filter::car_data::frame_index, \
        cv::Rect2d(p_tl, p_br), item.bbox, p_res));
    }


    return true;
}



