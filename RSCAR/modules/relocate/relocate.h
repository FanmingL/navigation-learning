/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : relocate.h
*   Author      : FanmingL
*   Created date: 2018-10-14 18:36:35
*   Description : 
*
*===============================================================*/


#ifndef _RELOCATE_H
#define _RELOCATE_H

#include <common/rs.h>
#include "common/io.h"
#include "common/main_interface.h"
#include "modules/relocate/relocate.pb.h"
#include "modules/detect/detect.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


namespace rs{
    namespace vp{
        class relocate : public common::rs{
        public:
            explicit relocate(const std::string &name);
            ~relocate() override = default;
            void Run() override ;
            bool ReadConfig();
            bool ReadData();
            void show_first_frame();
        private:
            RelocateConfig relocate_config;
            DetectVideo detect_video;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
        };
    }
}




#endif //RELOCATE_H
