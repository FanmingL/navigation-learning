//
// Created by erdou on 18-11-1.
//

#ifndef RSCAR_STABLE_DATA_H
#define RSCAR_STABLE_DATA_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include "common/rs.h"
#include "common/main_interface.h"
#include "common/io.h"
#include "common/image_util.h"

#include "modules/detect/detect.pb.h"
#include "modules/stabling/stabling.pb.h"

namespace rs {
    namespace vp {
        class stable_data : public common::rs {
        public:
            explicit stable_data(const std::string &name);

            ~stable_data() override = default;

            void Run() override;

            void ReadData();

            void AddCount();

            void init();

            cv::Mat GetFloatMatrix(const HomographMatrix &data);

            cv::Mat GetdoubleMatrix(const HomographMatrix &data);

        private:
            StablingConfig stabling_config;
            DetectVideo detect_in_data, detect_out_data;
            HomographVideo homograph_video;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
            cv::Rect max_rect;
            int step, count;
            cv::Mat frame;
            std::vector<cv::Mat> homograph_vector;

        };
    }
}

#endif //RSCAR_STABLE_DATA_H
