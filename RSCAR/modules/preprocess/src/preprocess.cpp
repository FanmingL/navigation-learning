/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : preprocess.cpp
*   Author      : FanmingL
*   Created date: 2018-10-10 18:52:48
*   Description : 
*
*===============================================================*/


#include "preprocess.h"

namespace rs{
    namespace vp{
        preprocess::preprocess(const std::string &name) : rs(name) {
            if (!ReadConfig())
                std::cout<<"Error Occurred While Loading config"<<std::endl;
            //std::cout<<preprocess_config.DebugString()<<std::endl;
            if (if_write_video())
            {
                std::remove((char*)common::get_absolute_path(preprocess_config.video_output_path()).c_str());
                video_writer.open(common::get_absolute_path(preprocess_config.video_output_path()),
                        CV_FOURCC('D', 'I', 'V', 'X'), preprocess_config.fps(),
                        cv::Size(preprocess_config.width_final(), preprocess_config.height_final()));
            }
                video_capture.open(common::get_absolute_path(preprocess_config.video_input_path()));

        }

        void preprocess::Run() {
            cv::Mat mat, res;
            cv::Rect ROI(preprocess_config.x_start(), preprocess_config.y_start(),
                    preprocess_config.width_cut(), preprocess_config.height_cut());
            int count = 0;
            while (true)
            {
                std::cout<<count++<<std::endl;
                video_capture >> mat;
                if (mat.empty())break;
                res = mat(ROI).clone();
                if (!(ROI.width == preprocess_config.width_final() && ROI.height == preprocess_config.height_final()))
                    cv::resize(res, res, cv::Size(preprocess_config.width_final(), preprocess_config.height_final()));
                if (preprocess_config.if_show_video())
                {
                    cv::imshow(GetModuleName(), res);
                    auto key = cv::waitKey(1);
                    if (key == 'q')break;
                }
            }
        }

        bool preprocess::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/preprocess/config/preprocess.prototxt", &preprocess_config);
        }

        bool preprocess::if_show_video() {
            return preprocess_config.if_show_video();
        }

        bool preprocess::if_write_video() {
            return preprocess_config.if_write_video();
        }


    }
}

MAIN(rs::vp::preprocess, "preprocess");

