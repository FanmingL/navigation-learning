/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : stabling.h
*   Author      : FanmingL
*   Created date: 2018-10-30 11:56:13
*   Description : 
*
*===============================================================*/


#ifndef _STABLING_H
#define _STABLING_H

#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"

#include "modules/stabling/stabling.pb.h"
#include "modules/detect/detect.pb.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#define USE_SIFT 1
namespace rs{
    namespace vp{
        class stabling : public common::rs{
        public:
            explicit stabling(const std::string & name);
            ~stabling() override = default;
            void Run() override;
            void ReadConfig();
            void AddCount();
            cv::Rect CvtData(const DetectObject &object);
            void init();
            void CalculateHomograph(const cv::Mat &src, const DetectFrame &detect_frame, cv::Mat &dst);
            void AddData(HomographMatrix * data, const cv::Mat &homograph_matrix);

        private:

            StablingConfig stabling_config;
            DetectVideo detect_video;
            cv::VideoCapture video_capture;
            cv::VideoWriter video_writer;
            cv::Rect max_rect, resize_rect;
            int count;
            cv::Mat label_mask;

#ifdef USE_SIFT
            cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> feature_detector;
            cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> descriptor;
#else
            cv::Ptr<cv::ORB> feature_detector;
            cv::Ptr<cv::xfeatures2d::LATCH> descriptor;
#endif
            cv::Mat last_frame, frame, canvas, description, last_description;
            std::vector<cv::KeyPoint> last_key_points;
            cv::BFMatcher bfMatcher;
            cv::Mat last_homograph;
            HomographVideo homograph_data;
            //cv::FlannBasedMatcher bfMatcher;
        };
    }
}
#endif //STABLING_H
