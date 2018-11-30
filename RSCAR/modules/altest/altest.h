/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : altest.h
*   Author      : FanmingL
*   Created date: 2018-10-18 13:53:24
*   Description : 
*
*===============================================================*/


#ifndef _ALTEST_H
#define _ALTEST_H

#include "common/rs.h"
#include "common/io.h"
#include "common/main_interface.h"
#include "modules/altest/altest_algorithm_base.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "modules/altest/altest.pb.h"

namespace rs {
    namespace vp {
        class altest : public common::rs {
        public:
            explicit altest(const std::string &name);

            ~altest() override = default;

            void Run() override;

            void ReadConfig();

        private:
            std::shared_ptr<AltestAlgorithmBase> algorithm;
            AltestConfig altest_config;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
            cv::Rect max_rect;
        };
    }
}


#endif //ALTEST_H
