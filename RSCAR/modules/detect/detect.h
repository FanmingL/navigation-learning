/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : detect.h
*   Author      : FanmingL
*   Created date: 2018-10-11 01:03:17
*   Description : 
*
*===============================================================*/


#ifndef _DETECT_H
#define _DETECT_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "modules/detect/detect.pb.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "modules/detect/detect_algorithm_base.h"
#include "common/string_util.h"

namespace rs {
    namespace vp {
        class detect : public common::rs {
        public:
            explicit detect(const std::string &name);

            ~detect() override = default;

            void Run() override;

            bool ReadConfig();

            bool ReadData();

        private:
            DetectVideo video_proto_data;
            cv::VideoCapture video_capture;
            DetectConfig detect_config;
            std::shared_ptr<DetectAlgorithmBase> detect_algorithm;

            void DrawOnImage(cv::Mat &src, cv::Mat &dst, const DetectFrame &frame);
        };
    }
}

#endif //DETECT_H
