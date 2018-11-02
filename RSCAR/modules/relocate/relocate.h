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
#include "common/image_util.h"
#include "common/main_interface.h"
#include "modules/relocate/relocate.pb.h"
#include "modules/detect/detect.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "modules/relocate/draw_area.h"
#include "common/string_util.h"

namespace rs {
    namespace vp {
        class relocate : public common::rs {
        public:
            explicit relocate(const std::string &name);

            ~relocate() override = default;

            void Run() override;

            bool ReadConfig();

            bool ReadData();

            void ShowFirstFrame();

            void CalculateHomographMatrix(cv::Mat &_homograph_matrix);

            void AddOneObject(const DetectObject &detect_object, const cv::Point2f &p_world, TrackObject *dst);

            void DrawOnImage(cv::Mat &inout_image, TrackObject *data);

        private:
            RelocateConfig relocate_config;
            PointPairSet points_pair_set;
            DetectVideo detect_video;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
            cv::Mat homograph_matrix;
            TrackVideo track_video;
            cv::Rect max_rect;
        };
    }
}


#endif //RELOCATE_H
