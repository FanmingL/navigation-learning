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

namespace rs {
    namespace vp {

        relocate::relocate(const std::string &name) : rs(name) {
            ReadConfig();
            ReadData();
            if (relocate_config.if_write_video()) {
                std::remove((char *) common::GetAbsolutePath(relocate_config.out_video_path()).c_str());
                video_writer.open(common::GetAbsolutePath(relocate_config.out_video_path()),
                                  CV_FOURCC('D', 'I', 'V', 'X'),
                                  30, cv::Size(relocate_config.width(), relocate_config.height()));
            }
            common::ReadProtoFromTextFile(relocate_config.point_data_path(), &points_pair_set);
            video_capture.open(common::GetAbsolutePath(relocate_config.input_video_path()));
            CalculateHomographMatrix(homograph_matrix);
            if (relocate_config.if_draw_mask()) {
                cv::Mat first_image;
                video_capture >> first_image;
                draw_area da(first_image, common::GetAbsolutePath(relocate_config.mask_save_path()),
                             relocate_config.width(), relocate_config.height());
            }
        }

        bool relocate::ReadConfig() {
            return common::ReadProtoFromTextFile("modules/relocate/config/relocate.prototxt", &relocate_config);
        }

        bool relocate::ReadData() {
            return common::ReadProtoFromBinaryFile(relocate_config.input_data_path(), &detect_video);
        }

        void relocate::Run() {
            cv::Mat src, dst;
            for (int counter = 0; counter < detect_video.frame_size(); counter++) {
                std::cout << counter << std::endl;
                if (relocate_config.if_show_video() || relocate_config.if_write_video())
                    video_capture >> src;
                auto frame_iter = track_video.add_frame();
                for (auto &item : detect_video.frame(counter).object()) {
                    cv::Point2f position_image, position_world;
                    if (item.name() == "person" || item.name() == "bicycle" || item.name() == "motorbike") {
                        position_image = cv::Point2f(item.x() + item.width() / 2, item.y() + item.height());
                    } else {
                        position_image = cv::Point2f(item.x() + item.width() / 2, item.y() + item.height() / 2);
                    }
                    common::CalculateTransform(homograph_matrix, position_image, position_world);
                    auto object_iter = frame_iter->add_object();
                    AddOneObject(item, position_world, object_iter);
                    if (relocate_config.if_show_video() || relocate_config.if_write_video())
                        DrawOnImage(src, object_iter);

                }
                if (relocate_config.if_write_video()) {
                    video_writer << src;
                }
                if (relocate_config.if_show_video()) {
                    cv::imshow("relocate", src);
                    auto key = cv::waitKey(1);
                    if (key == 'q')break;
                }

            }

            common::WriteProtoToBinaryFile(relocate_config.out_data_path(), &track_video);
            //for (int counter = 0;;counter++){
            //cv::Mat src;
            //video_capture >> src;
            //if (src.empty())break;
            //common::CalculateTransform();


            //}
        }

        void relocate::ShowFirstFrame() {
            cv::Mat mat;
            video_capture >> mat;
            for (auto &item : points_pair_set.points()) {
                cv::circle(mat, cv::Point(item.x_pixel(), item.y_pixel()), 2, cv::Scalar(0, 0, 255), -1);
                std::string text_on_pic;
                text_on_pic = std::to_string(item.x_world());
                cv::putText(mat, text_on_pic, cv::Point(item.x_pixel(), \
                        item.y_pixel()), cv::FONT_ITALIC, 0.4, cv::Scalar(255, 0, 0), 2);
                text_on_pic = std::to_string(item.y_world());
                cv::putText(mat, text_on_pic, cv::Point(item.x_pixel(), \
                        item.y_pixel() + 15), cv::FONT_ITALIC, 0.4, cv::Scalar(255, 0, 0), 2);
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
            cv::Mat tmp_val = cv::findHomography(image_points, world_points);
            _homograph_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    _homograph_matrix.at<float>(i, j) = (float) tmp_val.at<double>(i, j);
                }

        }

        void relocate::AddOneObject(const DetectObject &detect_object, const cv::Point2f &p_world, TrackObject *dst) {
            dst->set_probility(detect_object.probility());
            dst->set_name(detect_object.name());
            dst->set_image_bbox_height(detect_object.height());
            dst->set_image_bbox_width(detect_object.width());
            dst->set_image_bbox_x(detect_object.x());
            dst->set_image_bbox_y(detect_object.y());
            dst->set_object_index(detect_object.car_index());
            dst->set_world_position_x(p_world.x);
            dst->set_world_position_y(p_world.y);
        }

        void relocate::DrawOnImage(cv::Mat &inout_image, TrackObject *data) {
            cv::rectangle(inout_image, cv::Rect2f(data->image_bbox_x(), data->image_bbox_y(), data->image_bbox_width(),
                                                  data->image_bbox_height()),
                          cv::Scalar(255, 0, 0), 2);
            cv::putText(inout_image, common::to_string_with_precision(data->world_position_x(), 5),
                        cv::Point2f(data->image_bbox_x(), data->image_bbox_y()), cv::FONT_ITALIC, 0.4,
                        cv::Scalar(0, 0, 255), 2);
            cv::putText(inout_image, common::to_string_with_precision(data->world_position_y(), 5),
                        cv::Point2f(data->image_bbox_x(), data->image_bbox_y() + 15), cv::FONT_ITALIC, 0.4,
                        cv::Scalar(0, 0, 255), 2);

        }
    }
}


MAIN(rs::vp::relocate, "relocate");




