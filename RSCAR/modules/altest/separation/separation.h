/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : separation.h
*   Author      : FanmingL
*   Created date: 2018-10-18 22:14:51
*   Description : 
*
*===============================================================*/


#ifndef _SEPARATION_H
#define _SEPARATION_H

#include "common/io.h"
#include "common/algorithm_factory.h"
#include "modules/altest/altest_algorithm_base.h"
#include "modules/altest/separation/separation.pb.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
namespace rs{
    namespace vp{
        class separation : public AltestAlgorithmBase{
        public:
            separation();
            ~separation() = default;
            void PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) override;
            void ReadConfig();
            void RefineSegments(const cv::Mat &src, cv::Mat &mask, cv::Mat &dst);
            void MyRefine(cv::Mat &mask, int se_radius);
        private:
            SeparationConfig separation_config;
            cv::Mat bgmask;
            cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;

        };
        common::REGISTER_ALGORITHM(AltestAlgorithmBase,"separation",separation);
    }
}
#endif //SEPARATION_H
