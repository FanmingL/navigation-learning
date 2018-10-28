/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : information.cpp
*   Author      : FanmingL
*   Created date: 2018-10-15 18:20:01
*   Description : 
*
*===============================================================*/


#include <common/mean_filter.h>
#include "information.h"


namespace rs {
    namespace vp {

        information::information(const std::string &name) : rs(name) {
            ReadConfig();
            ReadData();
            ReadMask();
            ReadMatrix();
            MapInit();
            image_max_rect = cv::Rect(0, 0, information_config.width(), information_config.height());
        }

        void information::Run() {
            //video_data.frames(2).PrintDebugString();
            float distance_add_step = information_config.distance_step();
            std::unordered_map<int, std::shared_ptr<common::mean_filter<cv::Point2f> > > filter_map;
            std::unordered_map<int, DescriptionTrajectory *> trajectory_map;
            int max_index_now = -1;
            float distance_threshold = information_config.max_distance();
            cv::Mat frame_mask = mask.clone();
            for (int frame_index = 0; frame_index < video_data.frame_size(); frame_index++) {
                auto &frame = video_data.frame(frame_index);
                mask.copyTo(frame_mask);
                std::cout << frame_index << std::endl;
                if (frame_index > information_config.max_frame_count())break;
                if ( information_config.if_save_debug()  && frame_index % 100 == 99 ) {
                    std::ofstream of(common::GetAbsolutePath("data/debug.txt"), std::ios::trunc);
                    of << all_trajectories.trajectory(all_trajectories.trajectory_size()/2).DebugString();
                }
                std::unordered_map<int, cv::Point2f> position_after_filter;
                std::unordered_map<int, cv::Point2f> velocity;
                std::unordered_map<int, int> index_to_map;
                int last = -1;
                std::vector<std::pair<int, int> > object_index_sort;
                for (int j = 0; j < frame.object_size(); j++) {
                    object_index_sort.emplace_back(std::pair<int, int>(frame.object(j).object_index(), j));
                }
                std::sort(object_index_sort.begin(), object_index_sort.end());
                int j = 0;

                for (; j < frame.object().size(); j++) {

                    auto &item = frame.object(object_index_sort[j].second);
                    if (item.object_index() > max_index_now)break;
                    if (filter_map.count(item.object_index())) {
                        cv::Point2f point_tmp(item.world_position_x(), item.world_position_y());
                        //velocity[item.object_index()] = size_filter_map[item.object_index()].run(point_tmp, point_tmp) * 30.0;
                        cv::Point2f velocity_tmp;
                        point_tmp = filter_map[item.object_index()]->RunAndCalVel(point_tmp, velocity_tmp);
                        position_after_filter[item.object_index()] = point_tmp;
                        velocity[item.object_index()] = velocity_tmp * 30.0;
                        index_to_map[item.object_index()] = object_index_sort[j].second;
                    }
                }
                /// gap is the first who joins the trajectory
                int gap = j;

                std::vector<int> index_to_delete;
                for (auto &item : filter_map) {
                    if (!position_after_filter.count(item.first)) {
                        index_to_delete.push_back(item.first);
                    }
                }

                for (auto &item : index_to_delete) {
                    filter_map.erase(item);
                    trajectory_map.erase(item);
                }
                //std::cout<<"delete size: "<<index_to_delete.size()<<std::endl;
                //std::cout<<"remain size: "<<trajectory_map.size()<<std::endl;

                for (; j < frame.object().size(); j++) {
                    auto &item = frame.object(object_index_sort[j].second);
                    max_index_now = std::max(item.object_index(), max_index_now);

                    //cv::Point image_center = cv::Point((int)(item.image_position().x() + item.image_position().width()/2),
                    //                                   (int)(item.image_position().y() + item.image_position().height()/2));
                    cv::Point image_center = GetImageCenter(item);
                    //cv::Point image_tl = cv::Point((int)item.image_position().x(),(int)item.image_position().y());
                    //cv::Point image_br = cv::Point((int)(item.image_position().x() + item.image_position().width()),
                    //                                   (int)(item.image_position().y() + item.image_position().height()));
                    if (!(image_max_rect.contains(image_center) &&
                          mask.at<uchar>(image_center) != common::CANNOT_GO)
                            )
                        continue;
                    //size_filter_map[item.object_index()] = std::make_shared<move_mean>(cv::Point2d(item.world_center().x(),
                    //        item.world_center().y()), 4);
                    filter_map[item.object_index()] = std::make_shared<common::mean_filter<cv::Point2f> >(
                            cv::Point2f(item.world_position_x(),
                                        item.world_position_y()), information_config.mean_filter_length());
                    if (item.name() == "person")
                        trajectory_map[item.object_index()] = all_trajectories.add_trajectory();
                    position_after_filter[item.object_index()] = cv::Point2f(item.world_position_x(),
                                                                             item.world_position_y());
                    index_to_map[item.object_index()] = object_index_sort[j].second;
                }
                for (auto &item : position_after_filter) {
                    int car_index = index_to_map[item.first];
                    cv::Rect rect_fill_color_tmp(GetBoundingBox(frame.object(car_index)));
                    frame_mask(rect_fill_color_tmp & image_max_rect) = cv::Scalar(
                            string_image_map[frame.object(car_index).name()]);
                }

                for (auto &item : position_after_filter) {
                    if (!trajectory_map.count(item.first))continue;
                    auto iter = trajectory_map[item.first]->add_trajectory_point();
                    int car_index = index_to_map[item.first];
                    iter->set_image_bbox_x(frame.object(car_index).image_bbox_x());
                    iter->set_image_bbox_y(frame.object(car_index).image_bbox_y());
                    iter->set_image_bbox_width(frame.object(car_index).image_bbox_width());
                    iter->set_image_bbox_height(frame.object(car_index).image_bbox_height());
                    iter->set_frame_index(frame_index);
                    iter->set_world_position_x(item.second.x);
                    iter->set_world_position_y(item.second.y);
                    //std::cout<<frame.object(car_index).name()<<std::endl;
                    iter->set_object_type(string_map[std::string(frame.object(car_index).name())]);
                    //std::cout<<string_map[frame.object(car_index).name()]<<std::endl;
                    iter->set_object_index(frame.object(car_index).object_index());

                    double head_angle = 0;
                    if (velocity.count(item.first)) {
                        iter->set_world_velocity_x(velocity[item.first].x);
                        iter->set_world_velocity_y(velocity[item.first].y);
                        head_angle = atan2(velocity[item.first].y, velocity[item.first].x);
                    } else {
                        iter->set_world_velocity_x(-1000);
                        iter->set_world_velocity_y(-1000);
                    }
                    iter->set_angle_in_world((float) (head_angle / CV_PI * 180));
                    int it_index = index_to_map[item.first];
                    cv::Point it_point = GetImageCenter(frame.object(car_index));
                    cv::Point2f it_point_world(frame.object(it_index).world_position_x(),
                                               frame.object(it_index).world_position_y());
                    std::vector<cv::Point2f> point_image_obstacle(360), point_world_obstacle(360);
                    std::vector<std::pair<float, int> > angle_to_index(360);
                    std::vector<ObjectType> surround_type_image(360);
                    for (int angle_i = 0; angle_i < 360; angle_i++) {
                        auto angle_now_rad = (float) (angle_i * CV_PI / 180.0f);
                        float dist = 0;
                        cv::Point find_point = it_point + cv::Point((int) (dist * cosf(angle_now_rad)),
                                                                    (int) (dist * sinf(angle_now_rad)));
                        int recognize_step = 0;
                        common::MAP_TYPE original_data = (common::MAP_TYPE) frame_mask.at<uchar>(find_point);
                        for (;;) {
                            bool break_flag = false;
                            if (!image_max_rect.contains(find_point) || dist > distance_threshold) {
                                point_image_obstacle[angle_i] = cv::Point2f(find_point);
                                surround_type_image[angle_i] = NOTHING;
                                break;
                            }
                            auto val_here = frame_mask.at<uchar>(find_point);
                            switch (recognize_step) {
                                case 0:
                                    if ((int) val_here != (int) original_data)recognize_step = 1;
                                    break;
                                case 1:
                                    if (val_here != 255) {
                                        point_image_obstacle[angle_i] = cv::Point2f(find_point);
                                        switch (val_here) {
                                            case (int) common::CANNOT_GO:
                                                if (original_data != common::PERSON) {
                                                    surround_type_image[angle_i] = OBSTACLE;
                                                    break_flag = true;
                                                }
                                                break;
                                            case (int) common::CAR:
                                                surround_type_image[angle_i] = CAR;
                                                break_flag = true;
                                                break;
                                            case (int) common::PERSON:
                                                surround_type_image[angle_i] = PERSON;
                                                break_flag = true;
                                                break;
                                            case (int) common::BICYCLE:
                                                surround_type_image[angle_i] = BICYCLE;
                                                break_flag = true;
                                                break;
                                            case (int) common::MOTORBIKE:
                                                surround_type_image[angle_i] = MOTORBIKE;
                                                break_flag = true;
                                                break;
                                            case (int) common::NON_MOTOR_ROAD:
                                                if (original_data == common::CAR) {
                                                    surround_type_image[angle_i] = OBSTACLE;
                                                    break_flag = true;
                                                }
                                                break;
                                            default:
                                                break;
                                        }
                                    }
                                    break;
                            }
                            if (break_flag)break;
                            dist += distance_add_step;
                            find_point = it_point + cv::Point((int) (dist * cosf(angle_now_rad)),
                                                              (int) (dist * sinf(angle_now_rad)));
                        }


                    }
                    cv::Mat cal_tmp(3, 1, CV_32FC1, cv::Scalar(1));
                    cv::Mat res_tmp = cal_tmp.clone();
                    for (int angle_i = 0; angle_i < 360; angle_i++) {
                        cv::Point2f res_world_point_tmp;
                        common::CalculateTransform(homograph_matrix, point_image_obstacle[angle_i],
                                                   res_world_point_tmp);
                        auto relative_point_tmp = res_world_point_tmp - it_point_world;
                        float angle_tmp = atan2f(relative_point_tmp.y, relative_point_tmp.x) / (float) CV_PI * 180.0f;
                        point_world_obstacle[angle_i] = relative_point_tmp;
                        angle_to_index[angle_i] = std::pair<float, int>(angle_tmp, angle_i);
                    }
                    std::sort(angle_to_index.begin(), angle_to_index.end());
                    /// linear Interpolation !!!
                    int low_index = 0;
                    for (int angle_i = 0; angle_i < 360; angle_i++) {
                        auto iter_surround = iter->add_object_surround();

                        int angle_to_find = angle_i - 180 + 1;
                        iter_surround->set_angle(angle_to_find);
                        while (true) {
                            if (angle_to_index[low_index].first <= angle_to_find &&
                                angle_to_index[low_index + 1].first >= angle_to_find) {
                                int obj_index1 = angle_to_index[low_index].second,
                                        obj_index2 = angle_to_index[low_index + 1].second;
                                if (surround_type_image[obj_index1] == surround_type_image[obj_index2]) {
                                    cv::Point2f tmp_point_2f = (point_world_obstacle[obj_index1] *
                                                                (angle_to_find - angle_to_index[low_index].first)
                                                                + point_world_obstacle[obj_index2] *
                                                                  (angle_to_index[low_index + 1].first -
                                                                   angle_to_find)) /
                                                               (angle_to_index[low_index + 1].first -
                                                                angle_to_index[low_index].first);
                                    auto dist = (float) cv::norm(tmp_point_2f);
                                    iter_surround->set_distance(dist);
                                    iter_surround->set_type(surround_type_image[obj_index1]);
                                } else {
                                    float angle_diff_1 = -angle_to_index[low_index].first + angle_to_find,
                                            angle_diff_2 = angle_to_index[low_index + 1].first - angle_to_find;
                                    if (angle_diff_2 > angle_diff_1) {
                                        iter_surround->set_distance((float) cv::norm(
                                                point_world_obstacle[angle_to_index[low_index].second]));
                                        iter_surround->set_type(surround_type_image[
                                                                        angle_to_index[low_index].second]);
                                    } else {
                                        iter_surround->set_distance((float) cv::norm(
                                                point_world_obstacle[angle_to_index[low_index + 1].second]));
                                        iter_surround->set_type(surround_type_image[
                                                                        angle_to_index[low_index + 1].second]);
                                    }
                                }
                                break;
                            } else if (angle_to_index[low_index].first > angle_to_find) {
                                if (low_index > 0) {
                                    low_index--;
                                } else {
                                    iter_surround->set_distance((float) cv::norm(
                                            point_world_obstacle[angle_to_index[low_index].second]));
                                    iter_surround->set_type(surround_type_image[
                                                                    angle_to_index[low_index].second]);
                                    break;
                                }
                            } else if (angle_to_index[low_index + 1].first < angle_to_find) {
                                if (low_index + 1 < 359) {
                                    low_index++;
                                } else {
                                    iter_surround->set_distance((float) cv::norm(
                                            point_world_obstacle[angle_to_index[low_index + 1].second]));
                                    iter_surround->set_type(surround_type_image[
                                                                    angle_to_index[low_index + 1].second]);
                                    break;
                                }
                            }
                        }

                    }

                }
            }
            common::WriteProtoToBinaryFile(information_config.output_data_path(), &all_trajectories);
        }

        void information::ReadConfig() {
            common::ReadProtoFromTextFile("modules/information/config/information.prototxt", &information_config);
        }

        void information::ReadData() {
            common::ReadProtoFromBinaryFile(information_config.input_data_path(), &video_data);
        }

        void information::ReadMask() {
            mask = cv::imread(common::GetAbsolutePath(information_config.mask_path()), cv::IMREAD_GRAYSCALE);
        }

        void information::ReadMatrix() {
            PointPairSet points_pair_set;
            common::ReadProtoFromTextFile(information_config.pair_point_path(), &points_pair_set);
            std::vector<cv::Point2f> image_points, world_points;
            for (auto &item : points_pair_set.points()) {
                image_points.emplace_back(cv::Point2f(item.x_pixel(), item.y_pixel()));
                world_points.emplace_back(cv::Point2f(item.x_world(), item.y_world()));
            }
            cv::Mat tmp_val = cv::findHomography(image_points, world_points);
            homograph_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    homograph_matrix.at<float>(i, j) = (float) tmp_val.at<double>(i, j);
                }
        }

        void information::MapInit() {
            string_map["person"] = ObjectType::PERSON;
            string_map["car"] = ObjectType::CAR;
            string_map["bicycle"] = ObjectType::BICYCLE;
            string_map["motorbike"] = ObjectType::MOTORBIKE;
            string_image_map["person"] = common::PERSON;
            string_image_map["car"] = common::CAR;
            string_image_map["bicycle"] = common::BICYCLE;
            string_image_map["motorbike"] = common::MOTORBIKE;
        }

        cv::Point information::GetImageCenter(const TrackObject &object) {
            cv::Point image_center;
            if (object.name() == "car") {
                image_center = cv::Point((int) (object.image_bbox_x() + object.image_bbox_width() / 2),
                                         (int) (object.image_bbox_y() + object.image_bbox_height() / 2));
            } else {
                image_center = cv::Point((int) (object.image_bbox_x() + object.image_bbox_width() / 2),
                                         (int) (object.image_bbox_y() + object.image_bbox_height() - 1));
            }
            return image_center;
        }

        cv::Rect2f information::GetBoundingBox(const TrackObject &object) {
            cv::Rect2f rect;
            if (object.name() == "car"){
                rect = cv::Rect2f(object.image_bbox_x(), object.image_bbox_y(),
                        object.image_bbox_width(), object.image_bbox_height());
            } else{
                cv::Rect2f(object.image_bbox_x(), object.image_bbox_y() + object.image_bbox_height() *
                (information_config.den()-1)/information_config.den()
                                      , object.image_bbox_width(),
                                         1/information_config.den() * object.image_bbox_height());
                rect = cv::Rect2f(object.image_bbox_x(), object.image_bbox_y(),
                                  object.image_bbox_width(), object.image_bbox_height());
            }
            return rect;
        }
    }
}

MAIN(rs::vp::information, "information");

