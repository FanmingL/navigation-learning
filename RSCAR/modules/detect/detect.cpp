/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : detect.cpp
*   Author      : FanmingL
*   Created date: 2018-10-11 01:03:11
*   Description : 
*
*===============================================================*/


#include "select_algorithm.h"
#include "detect.h"

namespace rs{
    namespace vp{

        detect::detect(const std::string &name) : rs(name) {
            ReadConfig();
            ReadData();
            std::cout<<common::get_absolute_path(detect_config.video_input_path())<<std::endl;
            video_capture.open(common::get_absolute_path(detect_config.video_input_path()));
            detect_algorithm =
                    common::AlgorithmFactory<DetectAlgorithmBase>::CreateAlgorithm(detect_config.algorithm_name());
        }

        void detect::Run() {
            cv::Mat src, dst;
            std::vector<DetectData> res;
            while (true)
            {
                video_capture >> src;
                if (src.empty()){
                    std::cout<<"image cannot open"<<std::endl;
                    return ;
                }
                detect_algorithm->DetectObject(src, dst, res);
                cv::imshow(GetModuleName(), dst);
                auto key = cv::waitKey(30);
                if (key == 'q')break;
            }
            //std::cout<<video_proto_data.frame(1).DebugString()<<std::endl;
        }

        bool detect::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/detect/config/detect.prototxt", &detect_config);
        }

        bool detect::ReadData() {
            return common::ReadProtoFromBinaryFile("data/yolo_out_data.proto.b", &video_proto_data);
        }
    }
}


MAIN(rs::vp::detect, "detect");

