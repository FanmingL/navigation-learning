/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : filter_tracker.h
*   Author      : FanmingL
*   Created date: 2018-10-27 12:16:58
*   Description : 
*
*===============================================================*/


#ifndef _FILTER_TRACKER_H
#define _FILTER_TRACKER_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "common/image_util.h"
#include "common/util.h"
#include "common/mean_filter.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "modules/filter_tracker/filter_tracker.pb.h"
#include "modules/detect/detect.pb.h"

#include <iostream>
#include <unordered_map>

namespace rs {
    namespace vp {
        class tracker_filter : public common::rs {
        public:
            explicit tracker_filter(const std::string &name);

            ~tracker_filter() override = default;

            void Run() override;

            void ReadConfig();

            void AddCount();

            void ReadData();

            void DrawRect(cv::Mat &src, const DetectFrame &data);

            void NewData(const DetectFrame &in_data, DetectFrame *out_data, const DetectFrame &raw_data);

            void InitLength();

            void InitAreaRange();

        private:
            void InitColorMap();

            FilterTrackerConfig filter_tracker_config;
            cv::VideoCapture video_capture, mask_capture;
            cv::VideoWriter video_writer;
            cv::Mat frame, frame_mask;

            int count;
            DetectVideo in_video_data, out_video_data, raw_data;
            cv::Rect max_rect;
            cv::Rect2f max_rect_2f;

            std::unordered_map<std::string, cv::Scalar> color_map;
            std::unordered_map<std::string, float> area_low, area_high;
            std::unordered_map<int, std::shared_ptr<common::mean_filter<cv::Point2f> > > size_filter_map, pos_filter_map;
            std::unordered_map<int, int> length_map, motor_count, bicycle_count;
            /*  */
            //std::unordered_map<int,
        };
    }
}

#endif //FILTER_TRACKER_H
