//
// Created by erdou on 18-10-12.
//

#ifndef RSCAR_BASE_TRACK_ALGORITHM_H
#define RSCAR_BASE_TRACK_ALGORITHM_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <string>
#include <iostream>
#include <vector>
#include "modules/detect/detect.pb.h"


namespace rs {
    namespace vp {
        class TrackData {
        public:
            TrackData(const cv::Rect2f &rect = cv::Rect2f(0, 0, 0, 0), const std::string _name = std::string("1"),
                      int _object_index = 0, float _probility = 0.0f) : bbox(rect), name(_name),
                                                                        object_index(_object_index),
                                                                        probility(_probility) {}

            cv::Rect2f bbox;
            std::string name;
            float probility;
            int object_index;
        };

        class BaseTrackAlgorithm {
        public:
            BaseTrackAlgorithm() = default;

            virtual ~BaseTrackAlgorithm() = default;

            virtual void Track(const cv::Mat &src, cv::Mat &dst,
                               const DetectFrame &detect_frame, std::vector<TrackData> &res) = 0;

            void SetAlgorithmName(const std::string &_algorithm_name) { algorithm_name = _algorithm_name; }

            std::string GetAlgorithmName() { return algorithm_name; }

        private:
            std::string algorithm_name;
        };
    }
}

#endif //RSCAR_BASE_TRACK_ALGORITHM_H
