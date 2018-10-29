//
// Created by erdou on 18-10-25.
//

#include "mannul_test.h"

namespace rs {
    namespace vp {

        mannul_test::mannul_test(const std::string &name) : rs(name), count(0) {
            ReadConfig();
            video_capture.open(common::GetAbsolutePath(mannul_config.in_video_path()));
            color_map["person"] = cv::Scalar(128, 0, 0);
            color_map["bicycle"] = cv::Scalar(255, 255, 0);
            color_map["motorbike"] = cv::Scalar(0, 255, 0);
        }

        void mannul_test::Run() {
            cv::Mat frame, canvas;
            while (true) {
                auto path = GetTempPath(count);
                if (!common::ReadProtoFromBinaryFile(path, &detect_frame))break;
                video_capture >> frame;
                canvas = frame.clone();
                DrawRect(canvas, detect_frame);
                cv::imshow("test", canvas);
                if (cv::waitKey(100) == 'q')
                    break;

                AddCount();
            }
        }

        void mannul_test::ReadConfig() {
            common::ReadProtoFromTextFile("modules/mannul_mark/config/mannul_mark.prototxt", &mannul_config);
        }

        std::string mannul_test::GetTempPath(const int &_count) {
            std::string temp_path(mannul_config.out_data_path());
            temp_path += "/";
            temp_path += std::to_string(_count);
            temp_path += ".bin";
            return temp_path;
        }

        void mannul_test::AddCount() {
            std::cout << count++ << std::endl;
        }

        void mannul_test::DrawRect(cv::Mat &src, const DetectFrame &data) {
            for (auto &item : data.object()) {
                cv::Rect2f rect(item.x(), item.y(), item.width(), item.height());
                cv::rectangle(src, rect, color_map[item.name()], 2);
                cv::putText(src, std::to_string(item.car_index()), rect.tl(), cv::FONT_ITALIC, 1,
                            cv::Scalar(128, 0, 128), 2);
            }
        }

    }

}


MAIN(rs::vp::mannul_test, "mannul test");