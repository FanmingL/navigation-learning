//
// Created by erdou on 18-10-11.
//

#ifndef RSCAR_DETECT_ALGORITHM_BASE_H
#define RSCAR_DETECT_ALGORITHM_BASE_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <string>

namespace rs{
    namespace vp{
        class DetectData{
        public:
            explicit DetectData(const cv::Rect2f &_bbox=cv::Rect2f(0,0,0,0),
                    const std::string &_name = std::string("none"),
                    float _probility = 0.0f):
            bbox(_bbox), name(_name),probility(_probility){}
            cv::Rect2f bbox;
            std::string name;
            float probility;
        private:
        };
        class DetectAlgorithmBase{
        public:
            DetectAlgorithmBase() = default;
            virtual ~DetectAlgorithmBase() = default;
            virtual void DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res)=0;
            std::string GetAlgorithmName(){return algorithm_name;}
            void SetAlgorithmName(const std::string &_name){algorithm_name = _name;}
        private:
            std::string algorithm_name;

        };
    }
}

#endif //RSCAR_TRACKER_ALGORITHM_BASE_H
