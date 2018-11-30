//
// Created by erdou on 18-10-12.
//

#ifndef RSCAR_IMAGE_UTIL_H
#define RSCAR_IMAGE_UTIL_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace rs {
    namespace common {

        enum TF_TYPE {
            FLOAT_TYPE = 1,
            DOUBLE_TYPE = 2
        };

        void GetOneChannel(const cv::Mat &src, cv::Mat &dst, int channel_index = 0);

        void GetHue(const cv::Mat &src, cv::Mat &dst, int channel_index = 0);

        void CalculateTransform(const cv::Mat &transform_matrix, const cv::Point2f &src_point, cv::Point2f &dst_point,
                                int type = FLOAT_TYPE);

        float CalculateRectOverlapRatio(const cv::Rect2f &r1, const cv::Rect2f &r2, int mode = 2);

        enum OVERLAP_MODE {
            AND_OR = 1,
            AND_MIN = 2,
            AND_MAX = 3
        };

        enum MAP_TYPE {
            CANNOT_GO = 0,
            NON_MOTOR_ROAD = 20,
            MOTOR_ROAD = 40,
            PERSON_ROAD = 60,
            CAR = 80,
            PERSON = 100,
            BICYCLE = 120,
            MOTORBIKE = 140
        };
    }
}


#endif //RSCAR_IMAGE_UTIL_H
