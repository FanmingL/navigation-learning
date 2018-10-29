//
// Created by erdou on 18-10-25.
//

#ifndef RSCAR_TYPE_CVT_H
#define RSCAR_TYPE_CVT_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "modules/type_cvt/type_cvt.pb.h"
#include "modules/detect/detect.pb.h"


#include <fstream>
#include <iostream>
#include <opencv2/videoio.hpp>

namespace rs {
    namespace vp {
        class type_cvt : public common::rs {
        public:
            explicit type_cvt(const std::string &name);

            ~type_cvt() override = default;

            void Run() override;

            void ReadConfig();

        private:
            TypeCvtConfig type_config;
            cv::VideoCapture video_capture;
            std::ifstream iff;
            DetectVideo detect_video;

        };
    }
}

#endif //RSCAR_TYPE_CVT_H
