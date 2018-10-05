//
// Created by erdou on 18-10-3.
//

#ifndef ROAD_SIMULATION_STABLING_H
#define ROAD_SIMULATION_STABLING_H

#include "car_filter.h"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/calib3d.hpp"
#include <mutex>

#define USE_SIFT 1

class stabling {
public:
    stabling();

    static void onMouse(int event, int x, int y, int, void *user_data);

    void mouse_cb(int event, int x, int y, int flags);

    void init();

    void init_config();

    bool run(cv::Mat &dst);

    car_filter car_filter1;
    cv::Mat current_frame;
    std::vector<car_filter::car_data> current_car;
    const std::string window_name = std::string("stabling");
    const std::string init_window_name = std::string("init");
    std::string mask_path;
    cv::Mat mask;
private:
    std::ofstream of;

    void refresh_init(const cv::Mat &src);

#if USE_SIFT
    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift_feature_detector;
    cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> sift_descriptor;
#else
    cv::Ptr<cv::ORB> sift_feature_detector;
    cv::Ptr<cv::xfeatures2d::LATCH> sift_descriptor;
#endif
    std::vector<cv::KeyPoint> init_sift_key_points;
    cv::Mat init_canvas, init_description;
    cv::Mat init_image;
    cv::Mat homograph_from_init;
    std::mutex init_image_mutex;
    int click_step = 0;
    std::vector<cv::Point> click_point_buffer;
    cv::BFMatcher bfMatcher;
};


#endif //ROAD_SIMULATION_STABLING_H
