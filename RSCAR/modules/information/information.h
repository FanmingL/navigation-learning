/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : information.h
*   Author      : FanmingL
*   Created date: 2018-10-15 18:20:37
*   Description : 
*
*===============================================================*/


#ifndef _INFORMATION_H
#define _INFORMATION_H

#include "common/main_interface.h"
#include "common/io.h"
#include "common/rs.h"
#include "modules/track/track.pb.h"
#include "modules/relocate/relocate.pb.h"
#include "modules/information/information.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "common/image_util.h"
#include "opencv2/calib3d.hpp"
#include <unordered_set>
#include <unordered_map>
#include "common/util.h"

namespace rs {
    namespace vp {
        class information : public common::rs {
        public:
            explicit information(const std::string &name);

            ~information() override = default;

            void Run() override;

            void ReadConfig();

            void ReadData();

            void ReadMask();

            void ReadMatrix();

            void MapInit();

            cv::Point GetImageCenter(const TrackObject &object);

            cv::Rect2f GetBoundingBox(const TrackObject &object);

        private:
            TrackVideo video_data;
            AllTrajectory all_trajectories;
            InformationConfig information_config;
            cv::Mat mask, homograph_matrix;
            std::unordered_map<std::string, ObjectType> string_map;
            std::unordered_map<std::string, int> string_image_map;
            cv::Rect image_max_rect;

        };
    }
}

#endif //INFORMATION_H
