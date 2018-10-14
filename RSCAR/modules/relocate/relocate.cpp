/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : relocate.cpp
*   Author      : FanmingL
*   Created date: 2018-10-14 18:36:28
*   Description : 
*
*===============================================================*/


#include "relocate.h"

namespace rs{
    namespace vp{

        relocate::relocate(const std::string &name) : rs(name) {
            ReadConfig();
            if (relocate_config.if_write_video()){
                std::remove((char*)common::get_absolute_path(relocate_config.out_video_path()).c_str());
                video_writer.open(common::get_absolute_path(relocate_config.out_video_path()),CV_FOURCC('D', 'I', 'V', 'X'),
                        30,cv::Size(relocate_config.width(), relocate_config.height()));
            }
            video_capture.open(common::get_absolute_path(relocate_config.input_video_path()));
        }

        bool relocate::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/relocate/config/relocate.prototxt", &relocate_config);
        }

        bool relocate::ReadData() {
            return common::ReadProtoFromBinaryFile(relocate_config.input_data_path(), &detect_video);
        }

        void relocate::Run() {
            show_first_frame();
        }

        void relocate::show_first_frame() {
            cv::Mat mat;
            video_capture >> mat;
            cv::imshow("show first frame", mat);
            cv::waitKey();
            cv::destroyWindow("show first frame");
        }
    }
}


MAIN(rs::vp::relocate, "relocate");




