//
// Created by erdou on 18-10-25.
//

#ifndef RSCAR_MANNUL_TEST_H
#define RSCAR_MANNUL_TEST_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


#include "modules/detect/detect.pb.h"
#include "modules/mannul_mark/mannul_mark.pb.h"
#include <unordered_map>

namespace rs {
    namespace vp {
        class mannul_test : public common::rs {
        public:
            explicit mannul_test(const std::string &name);

            ~mannul_test() override = default;

            void Run() override;

            void ReadConfig();

            void AddCount();

            std::string GetTempPath(const int &_count);

            void DrawRect(cv::Mat &src, const DetectFrame &data);

        private:
            DetectFrame detect_frame;
            MannulConfig mannul_config;
            int count;
            cv::VideoCapture video_capture;
            std::unordered_map<std::string, cv::Scalar> color_map;


        };
    }
}

#endif //RSCAR_MANNUL_TEST_H
