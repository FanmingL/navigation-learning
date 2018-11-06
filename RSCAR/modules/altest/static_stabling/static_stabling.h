//
// Created by erdou on 18-11-4.
//

#ifndef RSCAR_STATIC_STABLING_H
#define RSCAR_STATIC_STABLING_H

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "common/image_util.h"
#include "common/string_util.h"

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
            void MyRefine(cv::Mat &mask, int radius, std::vector<cv::Rect2f> &bboxs, std::vector<cv::Point2f> &mass_center, std::vector<float> &area);
            /// P,theta
            void MyHoughLine(const std::vector<cv::Point2f> &points, const int &threshold, std::vector<cv::Vec2f>& lines);
            void GetPointsPair(std::vector<std::vector<cv::Point2f> > &points, const cv::Mat &gray, const cv::Rect2f &roi, const cv::Mat& mask = cv::Mat());
            void Match(const std::vector<std::vector<cv::Point2f> > &points);
            cv::Point2f GetMassCenter(const std::vector<cv::Point> &points);
        private:
            StaticStablingConfig config;
            cv::Rect max_rect, roi, show_roi;
            cv::Rect2f roi2f;
            cv::Mat mask, bgmask;
            bool init_flag;
#define USE_KNN
#ifdef USE_KNN
            cv::Ptr<cv::BackgroundSubtractorKNN> bgsubtractor;
#else
            cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;
#endif
            //cv::Ptr<cv::BackgroundSubtractor> bgsubtractor;
            cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> feature_detector;
            cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> descriptor;
            cv::Mat homograph_matrix;
            std::vector<std::vector<cv::Point2f> > init_points;
            std::vector<cv::Scalar> colors;
        };
        common::REGISTER_ALGORITHM(AltestAlgorithmBase, "stabling", static_stabling);
    }
}


#endif //RSCAR_STATIC_STABLING_H
