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
            if (detect_config.if_show_directly())
                ReadData();
            std::cout<<common::get_absolute_path(detect_config.video_input_path())<<std::endl;
            video_capture.open(common::get_absolute_path(detect_config.video_input_path()));
            detect_algorithm =
                    common::AlgorithmFactory<DetectAlgorithmBase>::CreateAlgorithm(detect_config.algorithm_name());
        }

        void detect::Run() {
            cv::Mat src, dst;
            std::vector<DetectData> res;
            int count = 0;
            while (true)
            {
                video_capture >> src;
                if (src.empty()){
                    return ;
                }
                if (!detect_config.if_show_directly()) {
                    detect_algorithm->DetectObject(src, dst, res);
                }else{
                    DrawOnImage(src,dst,video_proto_data.frame(count));;
                }
                cv::imshow(GetModuleName(), dst);
                auto key = cv::waitKey(1);
                if (key == 'q')break;
                std::cout<<count++<<std::endl;
            }
            //std::cout<<video_proto_data.frame(1).DebugString()<<std::endl;
        }

        bool detect::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/detect/config/detect.prototxt", &detect_config);
        }

        bool detect::ReadData() {
            return common::ReadProtoFromBinaryFile(detect_config.data_input_path(), &video_proto_data);
        }

        void detect::DrawOnImage(cv::Mat &src, cv::Mat &dst, const DetectFrame &frame) {
            cv::Mat _tmp = src.clone();
            for (auto &item : frame.object()) {
                if (item.probility() < detect_config.direct_threshold())continue;
                cv::Rect2f rect(cv::Rect2f((item.x() - item.width() / 2) * src.cols,
                                               (item.y() - item.height() / 2) * src.rows, item.width() * src.cols,
                                               item.height() * src.rows));
                cv::rectangle(_tmp, rect, cv::Scalar(0, 128, 0), 2);
                std::string t;
                t+=common::to_string_with_precision(item.probility()*100,4);
                cv::putText(_tmp,t,rect.tl(),cv::FONT_ITALIC,1,cv::Scalar(0,0,128), 2);
            }
            dst = _tmp.clone();
        }
    }
}


MAIN(rs::vp::detect, "detect");

