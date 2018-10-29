//
// Created by erdou on 18-10-25.
//

#include "type_cvt.h"

namespace rs {
    namespace vp {

        type_cvt::type_cvt(const std::string &name) : rs(name) {
            ReadConfig();
            video_capture.open(common::GetAbsolutePath(type_config.input_mask_video_path()));
            iff.open(common::GetAbsolutePath(type_config.input_data_path()));
        }

        void type_cvt::Run() {
            cv::Rect max_rect(0, 0, 1080, 1080);
            while (true) {
                std::string tmp_str1;
                if (!std::getline(iff, tmp_str1))break;
                auto *iter = detect_video.add_frame();
                int num = 0;
                std::stringstream ss1(tmp_str1);
                ss1 >> num;
                for (int i = 0; i < num; i++) {
                    auto *iter2 = iter->add_object();
                    std::string tmp_str2;
                    std::getline(iff, tmp_str2);
                    std::stringstream ss2(tmp_str2);
                    int x1, y1, x2, y2;
                    std::string name;
                    ss2 >> x1 >> y1 >> x2 >> y2 >> name;
                    cv::Rect bbox(cv::Point(x1, y1), cv::Point(x2, y2));
                    bbox = bbox & max_rect;
                    iter2->set_x(bbox.x);
                    iter2->set_y(bbox.y);
                    iter2->set_width(bbox.width);
                    iter2->set_height(bbox.height);
                    iter2->set_probility(100);
                    iter2->set_name(name);
                    //iter2->PrintDebugString();
                }

            }
            std::cout << detect_video.frame_size() << std::endl;
            common::WriteProtoToBinaryFile(type_config.output_data_path(), &detect_video);
        }

        void type_cvt::ReadConfig() {
            common::ReadProtoFromTextFile("modules/type_cvt/config/type_cvt.prototxt", &type_config);
            type_config.PrintDebugString();
        }

    }
}


MAIN(rs::vp::type_cvt, "type cvt");
