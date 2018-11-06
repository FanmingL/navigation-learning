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
            max_rect = cv::Rect2d(0,0,video_capture.get(cv::CAP_PROP_FRAME_WIDTH),
                    video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));
            altest_config.PrintDebugString();
            if (altest_config.if_write_video()){
                std::remove(common::GetAbsolutePath(altest_config.out_video_path()).c_str());
            }
        }

        void altest::Run() {
            cv::Mat dst, src;
            bool init_flag = false;
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
                if (altest_config.if_write_video()){
                    if (!init_flag){
                       video_writer.open(common::GetAbsolutePath(altest_config.out_video_path()),CV_FOURCC('D','I','V','X'),
                               30,dst.size()
                               );
                       init_flag = true;
                    }
                    video_writer << dst;
                }
            }
        }

        void altest::ReadConfig() {
            common::ReadProtoFromTextFile("modules/altest/config/altest.prototxt", &altest_config);
        }
    }
}


MAIN(rs::vp::altest, "altest");