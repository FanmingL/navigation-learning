//
// Created by erdou on 18-11-4.
//

#ifndef RSCAR_STATIC_STABLING_H
#define RSCAR_STATIC_STABLING_H

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "common/image_util.h"

#include "modules/altest/altest_algorithm_base.h"
#include "modules/altest/static_stabling/static_stabling.pb.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <iostream>

namespace rs{
    namespace vp{
        class static_stabling : public AltestAlgorithmBase{
        public:
            static_stabling();
            ~static_stabling() override = default;
            void PerformAlgorithm(const cv::Mat &src, cv::Mat &dst)override;
            void ReadConfig();
            void init(const cv::Mat &src);
            bool if_need_init();
            void MyRefine(cv::Mat &mask, int radius, std::vector<cv::Rect2f> &bboxs);
            /// P,theta
            void MyHoughLine(const std::vector<cv::Point2f> &points, const int threshold, std::vector<cv::Vec2f>& lines);
        private:
            StaticStablingConfig config;
            cv::Rect max_rect, roi;
            cv::Rect2f roi2f;
            cv::Mat mask, bgmask;
            bool init_flag;
            cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;
            cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> feature_detector;
            cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> descriptor;

        };
        common::REGISTER_ALGORITHM(AltestAlgorithmBase, "stabling", static_stabling);
    }
}


#endif //RSCAR_STATIC_STABLING_H
