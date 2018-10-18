/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : altest.cpp
*   Author      : FanmingL
*   Created date: 2018-10-18 13:53:17
*   Description : 
*
*===============================================================*/


#include "modules/altest/altest.h"

#include "modules/altest/altest_select_algorithm.h"
#include "altest.h"


namespace rs{
    namespace vp{

        altest::altest(const std::string &name) : rs(name) {
            ReadConfig();
            algorithm = common::AlgorithmFactory<AltestAlgorithmBase>::CreateAlgorithm(altest_config.algorithm_name());
            video_capture.open(common::GetAbsolutePath(altest_config.input_video_path()));
            altest_config.PrintDebugString();
        }

        void altest::Run() {
            cv::Mat dst, src;
            while (true){
                video_capture >> src;
                if (src.empty())break;
                algorithm->PerformAlgorithm(src, dst);
                if (altest_config.if_show_video()){
                    cv::imshow("altest", dst);
                    auto key = cv::waitKey(1);
                    if (key == 'q')
                        break;
                }
            }
        }

        void altest::ReadConfig() {
            common::ReadProtoFromTextFile("modules/altest/config/altest.prototxt", &altest_config);
        }
    }
}


MAIN(rs::vp::altest, "altest");