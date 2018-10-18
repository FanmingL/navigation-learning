/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : lkopticalflow.h
*   Author      : FanmingL
*   Created date: 2018-10-18 13:58:59
*   Description : 
*
*===============================================================*/


#ifndef _LKOPTICALFLOW_H
#define _LKOPTICALFLOW_H

#include "modules/altest/altest_algorithm_base.h"
#include "common/algorithm_factory.h"
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "modules/altest/lkopticalflow/lkopticalflow.pb.h"
#include "common/io.h"
#include "opencv2/optflow.hpp"
namespace rs{
    namespace vp{
        class LkOpticalFlow : public AltestAlgorithmBase{
        public:
            explicit LkOpticalFlow();
            ~LkOpticalFlow() override = default;
            void PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) override;
            void ReadConfig();

        private:
            LkConfig lk_config;
            bool need_init;
            cv::Mat last;
        };

       common::REGISTER_ALGORITHM(AltestAlgorithmBase, "lkopticalflow", LkOpticalFlow);
    }
}


#endif //LKOPTICALFLOW_H
