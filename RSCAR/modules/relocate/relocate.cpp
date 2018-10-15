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
            common::ReadProtoFromTextFile(relocate_config.point_data_path(), &points_pair_set);
            video_capture.open(common::get_absolute_path(relocate_config.input_video_path()));
            CalculateHomographMatrix(homograph_matrix);
            if (relocate_config.if_draw_mask()){
                cv::Mat first_image;
                video_capture >> first_image;
                draw_area da(first_image, common::get_absolute_path(relocate_config.mask_save_path()),
                        relocate_config.width(),relocate_config.height());
            }
        }

        bool relocate::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/relocate/config/relocate.prototxt", &relocate_config);
        }

        bool relocate::ReadData() {
            return common::ReadProtoFromBinaryFile(relocate_config.input_data_path(), &detect_video);
        }

        void relocate::Run() {
            return ;
            for (int counter = 0;;counter++){
                cv::Mat src;
                video_capture >> src;
                if (src.empty())break;



            }
        }

        void relocate::ShowFirstFrame() {
            cv::Mat mat;
            video_capture >> mat;
            for (auto &item : points_pair_set.points()) {
                cv::circle(mat, cv::Point(item.x_pixel(), item.y_pixel()), 2, cv::Scalar(0, 0, 255), -1);
                std::string text_on_pic;
                text_on_pic = std::to_string(item.x_world());
                cv::putText(mat, text_on_pic, cv::Point(item.x_pixel(),\
                        item.y_pixel()), cv::FONT_ITALIC, 0.4, cv::Scalar(255,0,0),2);
                text_on_pic = std::to_string(item.y_world());
                cv::putText(mat, text_on_pic, cv::Point(item.x_pixel(),\
                        item.y_pixel() + 15), cv::FONT_ITALIC, 0.4, cv::Scalar(255,0,0),2);
            }
            cv::imshow("show first frame", mat);
            cv::waitKey();
            cv::destroyWindow("show first frame");
        }

        void relocate::CalculateHomographMatrix(cv::Mat &_homograph_matrix) {
            std::vector<cv::Point2f> image_points, world_points;
            for (auto &item : points_pair_set.points()) {
                image_points.emplace_back(cv::Point2f(item.x_pixel(), item.y_pixel()));
                world_points.emplace_back(cv::Point2f(item.x_world(), item.y_world()));
            }
            _homograph_matrix = cv::findHomography(image_points, world_points);
        }
    }
}


MAIN(rs::vp::relocate, "relocate");




