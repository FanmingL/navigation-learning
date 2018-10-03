//
// Created by erdou on 18-10-3.
//

#include <src/stabling/include/stabling.h>

#include "stabling.h"
stabling::stabling():
sift_feature_detector(cv::xfeatures2d::SiftFeatureDetector::create()),
sift_descriptor(cv::xfeatures2d::SiftDescriptorExtractor::create()),
bfMatcher(cv::NORM_L2)
{
    mask_path = car_filter1.base_path + "/data/stabling_mask.jpg";
    mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE) >= 100;
    /*cv::imshow(init_window_name,mask);
    cv::waitKey(0);*/
    init();
}


void stabling::init_config() {
    car_filter1.read_next_frame(current_car, current_frame);
    mask = cv::Mat(cv::Size(current_frame.rows, current_frame.cols), CV_8UC1, cv::Scalar(255));
    click_point_buffer = std::vector<cv::Point>(2);
    for (auto &item : current_car)
        mask(item.bbox) = cv::Scalar(0);
    refresh_init(current_frame);
    cv::imshow(init_window_name, init_canvas);
    cv::setMouseCallback(init_window_name, stabling::onMouse, this);
    while (true)
    {
        {
            std::unique_lock<std::mutex> unique_lock(init_image_mutex);
            cv::imshow(init_window_name, init_canvas);
        }
        auto key = cv::waitKey(30);
        if (key == 'q')break;
    }
    auto mask_path = car_filter1.base_path + "/data/stabling_mask.jpg";
    std::remove((char*)(mask_path).c_str());
    cv::imwrite(mask_path,mask);
}

void stabling::mouse_cb(int event, int x, int y, int flags) {
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            std::cout<<x<<", "<<y<<std::endl;
            click_point_buffer[click_step] = cv::Point(x, y);
            if (click_step)
            {
                std::unique_lock<std::mutex> unique_lock(init_image_mutex);
                cv::Rect rect_tmp(click_point_buffer[0], click_point_buffer[1]);
                mask(rect_tmp) = cv::Scalar(0);
                refresh_init(current_frame);
            }
            click_step = 1 - click_step;
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        default:
            break;
    }

}

void stabling::onMouse(int event, int x, int y, int flag, void *user_data) {
    auto* temp = reinterpret_cast<stabling*>(user_data);
    temp->mouse_cb(event, x, y, flag);
}

void stabling::refresh_init(const cv::Mat &src) {
    init_canvas = cv::Scalar(0,0,0);
    src.copyTo(init_canvas,mask);
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    sift_feature_detector->detect(gray, init_sift_key_points, mask);
    sift_descriptor->compute(gray,init_sift_key_points, init_description);
    cv::drawKeypoints(init_canvas,init_sift_key_points,init_canvas,cv::Scalar(255,0,0));
}

void stabling::init() {
    car_filter1.read_next_frame(current_car, current_frame);
    refresh_init(current_frame);
    current_frame.copyTo(init_image);
}

bool stabling::run(cv::Mat &dst) {
    if (!car_filter1.read_next_frame(current_car,current_frame))return false;
    cv::Mat it_mask(current_frame.rows, current_frame.cols,CV_8UC1, cv::Scalar(255));
    /*for (auto &item : current_car)
        if (item.bbox.width > 0 && item.bbox.height > 0 )
            it_mask(item.bbox) = cv::Scalar(0);*/
    std::vector<cv::KeyPoint> sift_key_points;
    cv::Mat sift_description;
    cv::Mat res_img, gray;
    std::vector<cv::DMatch> good_matches;
    cv::cvtColor(current_frame, gray, CV_BGR2GRAY);
    sift_feature_detector->detect(gray, sift_key_points, it_mask);
    sift_descriptor->compute(gray,sift_key_points, sift_description);
    /// query train
    bfMatcher.match(init_description, sift_description, good_matches);
    int match_num = 50;
    std::nth_element(good_matches.begin(), good_matches.begin()+match_num-1, good_matches.end());
    good_matches.erase(good_matches.begin()+match_num, good_matches.end());
    //cv::drawMatches(init_image,init_sift_key_points,current_frame,sift_key_points,good_matches, dst);
    std::vector<cv::Point2f> init_points, now_points;
    for (auto & item : good_matches)
    {
        init_points.push_back(init_sift_key_points[item.queryIdx].pt);
        now_points.push_back(sift_key_points[item.trainIdx].pt);
    }
    homograph_from_init = cv::findHomography(now_points, init_points, cv::RANSAC);
    cv::warpPerspective(current_frame, dst, homograph_from_init, current_frame.size());
    std::cout<<car_filter::car_data::frame_index<<std::endl;

}


