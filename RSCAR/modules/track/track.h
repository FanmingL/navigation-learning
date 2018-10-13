/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : track.h
*   Author      : FanmingL
*   Created date: 2018-10-12 13:21:53
*   Description : 
*
*===============================================================*/


#ifndef _TRACK_H
#define _TRACK_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "modules/track/base_track_algorithm.h"
#include "modules/track/track.pb.h"
#include "modules/detect/detect.pb.h"
#include <string>


namespace rs{
    namespace vp{
        class track :public rs::common::rs{
        public:
            explicit track(const std::string &_name);
            ~track() override = default;
            void Run() override;
            void ReadConfig();
            void ReadData();
            void AddObject(const TrackData &data, DetectObject *object);

        private:
            std::shared_ptr<BaseTrackAlgorithm> track_algorithm;
            TrackConfig track_config;
            DetectVideo detect_data;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
            int counter = 0;
            DetectVideo detect_video;
        };
    }
}
#endif //TRACK_H
