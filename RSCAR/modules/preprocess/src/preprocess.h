/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : preprocess.h
*   Author      : FanmingL
*   Created date: 2018-10-10 18:53:18
*   Description : 
*
*===============================================================*/


#ifndef _PREPROCESS_H
#define _PREPROCESS_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "modules/preprocess/preprocess.pb.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rs {
    namespace vp {
        class preprocess : public rs::common::rs{
        public:
            preprocess(const std::string & name);
            ~preprocess() final = default;
            void Run() override;
            bool ReadConfig();
            bool if_show_video();
            bool if_write_video();
        private:
            PreprocessConfig preprocess_config;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
        };
    }
}
#endif //PREPROCESS_H
