/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : mannul_mark.cpp
*   Author      : FanmingL
*   Created date: 2018-10-24 22:32:38
*   Description : 
*
*===============================================================*/


#include "mannul_mark.h"


namespace rs {
    namespace vp {

        mannul_mark::mannul_mark(const std::string &name) : rs(name), count(0), step_flag(false),
                                                            main_program_run_flag(true), key(-1),
                                                            mark_state(INIT_FRAME), object_index(0),
                                                            object_type(PERSON) {
            ReadConfig();
            ReadData();
            video_capture.open(common::GetAbsolutePath(mannul_config.in_video_path()));
            width = (int) video_capture.get(cv::CAP_PROP_FRAME_WIDTH);
            height = (int) video_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
            init();
        }

        void mannul_mark::Run() {
            DetectFrame data_frame;
            cv::Mat last_canvas;
            cv::namedWindow(canvas_window_name);
            cv::setMouseCallback(canvas_window_name, mannul_mark::onMouse, this);
            std::thread key_monitor(&mannul_mark::GetKeyAct, this);
            key_monitor.detach();
            std::unordered_map<int, cv::Ptr<cv::TrackerKCF> > trackers;
            std::unordered_map<int, std::string> trackers_name;
            while (true) {
                step_flag = false;
                data_frame = detect_video.frame(count);
                video_capture >> frame;
                canvas = frame.clone();
                SetFrameData(data_frame);
                DrawRectangle(canvas);
                if (frame.empty())break;
                /*
                for (auto &item : trackers){
                    int index_tmp = item.first;
                    cv::Rect2d bbox_tmp;
                    if (item.second->update(frame, bbox_tmp)){
                        frame_data_list.emplace_back(data_set(TrackData(cv::Rect2f(bbox_tmp),
                                trackers_name[index_tmp],index_tmp,100), true));
                    }
                }
                trackers.clear();
                trackers_name.clear();*/
                while (!step_flag) {
                    {
                        std::lock_guard<std::mutex> lock_guard_(show_image_mutex);
                        canvas = frame.clone();
                        DrawRectangle(canvas);
                        cv::imshow(canvas_window_name, canvas);
                    }
                    key = cv::waitKey(1);
                    if (key == ' ' || key == '\\')step_flag = true;
                    switch (key) {
                        case '1':
                            object_type = PERSON;
                            break;
                        case '2':
                            object_type = BICYCLE;
                            break;
                        case '3':
                            object_type = MOTORBIKE;
                            break;
                        default:
                            break;
                    }
                    switch (mark_state) {
                        case INIT_FRAME: {
                            point_buffer.clear();
                            switch (key) {
                                case 'q'://NEW BBOX
                                    mark_state = NEW_BBOX;
                                    break;
                                case 'a':
                                    mark_state = CHANGE_BBOX;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                        case CHANGE_BBOX: {
                            switch (key) {
                                case 'q': {
                                    point_buffer.clear();
                                    mark_state = NEW_BBOX;
                                    break;
                                }
                                case 's': {
                                    if (point_buffer.size() < 2)break;
                                    cv::Rect2f tmp(point_buffer[0], point_buffer[1]);
                                    for (auto iter = frame_data_list.begin(); iter != frame_data_list.end();) {
                                        if (common::CalculateRectOverlapRatio(iter->track_data.bbox, tmp,
                                                                              common::AND_OR) > 0.0)
                                            iter = frame_data_list.erase(iter);
                                        else
                                            iter++;
                                    }
                                    point_buffer.clear();
                                    break;
                                }
                                case 'd':
                                    if (!point_buffer.empty())
                                        point_buffer.pop_back();
                                    break;

                                default:
                                    break;
                            }
                            break;
                        }
                        case NEW_BBOX: {
                            switch (key) {
                                case 'a': {
                                    point_buffer.clear();
                                    mark_state = CHANGE_BBOX;
                                    break;
                                }
                                case 'r': { // add new person
                                    if (point_buffer.size() < 2)break;
                                    frame_data_list.emplace_back(data_set(
                                            TrackData(cv::Rect(point_buffer[0], point_buffer[1]),
                                                      GetStringFromType(object_type), object_index++, 100.0), true));
                                    point_buffer.clear();
                                    break;
                                }
                                case 'w': {//
                                    if (point_buffer.size() < 2)break;
                                    int index = -1;
                                    float max_overlap = 0;
                                    std::string name_it_tmp;
                                    cv::Rect rect_tmp(point_buffer[0], point_buffer[1]);
                                    for (auto &item : last_list) {
                                        float overlap = common::CalculateRectOverlapRatio(item.track_data.bbox,
                                                                                          rect_tmp, common::AND_OR);
                                        if (overlap > max_overlap) {
                                            index = item.track_data.object_index;
                                            max_overlap = overlap;
                                            name_it_tmp = item.track_data.name;
                                        }
                                    }
                                    if (index >= 0) {
                                        frame_data_list.emplace_back(data_set(
                                                TrackData(cv::Rect(point_buffer[0], point_buffer[1]),
                                                          name_it_tmp, index, 100.0),
                                                true));
                                    } else {
                                        frame_data_list.emplace_back(data_set(
                                                TrackData(cv::Rect(point_buffer[0], point_buffer[1]),
                                                          GetStringFromType(object_type), object_index++, 100.0),
                                                true));
                                    }
                                    point_buffer.clear();
                                    break;
                                }
                                case 'e': {
                                    if (!point_buffer.empty())
                                        point_buffer.pop_back();
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
                last_list = frame_data_list;
                /*
                for (auto &item:frame_data_list){
                    trackers[item.track_data.object_index] = cv::TrackerKCF::create();
                    trackers[item.track_data.object_index]->init(frame,item.track_data.bbox);
                    trackers_name[item.track_data.object_index] = item.track_data.name;
                }*/
                SaveToFile();
                frame_data_list.clear();
                mark_state = INIT_FRAME;
                if (key == '\\')break;
                last_canvas = canvas.clone();
                cv::imshow("last_image", last_canvas);
                AddCount();
            }
        }

        void mannul_mark::ReadConfig() {
            common::ReadProtoFromTextFile("modules/mannul_mark/config/mannul_mark.prototxt", &mannul_config);
            mannul_config.PrintDebugString();

        }

        void mannul_mark::ReadData() {
            if (common::ReadProtoFromBinaryFile(mannul_config.in_data_path(), &detect_video))
                std::cout << "load data ok\n";
        }


        void mannul_mark::onMouse(int event, int x, int y, int flag, void *user_data) {
            mannul_mark *temp = reinterpret_cast<mannul_mark *>(user_data);
            temp->MouseCb(event, x, y, flag);
        }

        void mannul_mark::MouseCb(int event, int x, int y, int flags) {
            std::lock_guard<std::mutex> lock_guard_(show_image_mutex);
            x_now = x;
            y_now = y;
            switch (event) {
                case CV_EVENT_LBUTTONDOWN: {
                    switch (mark_state) {
                        case NEW_BBOX: {
                            if (point_buffer.empty()) {
                                point_buffer.emplace_back(cv::Point(x, y));
                            } else if (point_buffer.size() == 1) {
                                point_buffer.emplace_back(cv::Point(x, y));
                            }
                            break;
                        }
                        case CHANGE_BBOX: {
                            if (point_buffer.empty()) {
                                point_buffer.emplace_back(cv::Point(x, y));
                            } else if (point_buffer.size() == 1) {
                                point_buffer.emplace_back(cv::Point(x, y));
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    std::cout << x << ", " << y << std::endl;
                    break;
                }
                case CV_EVENT_MOUSEMOVE:
                    break;
                case CV_EVENT_LBUTTONUP:
                    break;
                default:
                    break;
            }
        }

        void mannul_mark::AddCount() {
            std::cout << count++ << std::endl;
        }

        void mannul_mark::DrawRectangle(cv::Mat &src) {
            for (auto &item : frame_data_list) {
                if (item.track_data.name == "person") {
                    cv::rectangle(src, item.track_data.bbox, cv::Scalar(0, 0, 255), 1);
                } else if (item.track_data.name == "bicycle") {
                    cv::rectangle(src, item.track_data.bbox, cv::Scalar(255, 0, 0), 1);
                } else {
                    cv::rectangle(src, item.track_data.bbox, cv::Scalar(255, 0, 144), 1);
                }
                cv::putText(src, std::to_string(item.track_data.object_index), (item.track_data.bbox.tl()
                                                                                + item.track_data.bbox.br()) / 2.0,
                            cv::FONT_ITALIC, 0.8, cv::Scalar(0, 0, 128), 1);
            }
            cv::putText(src, GetStringFromType(object_type), cv::Point(30, 30), cv::FONT_ITALIC, 1,
                        cv::Scalar(255, 255, 255), 2);
            cv::putText(src, GetStringFromState(mark_state), cv::Point(30, 70), cv::FONT_ITALIC, 1,
                        cv::Scalar(255, 255, 255), 2);
            cv::putText(src, std::to_string(count), cv::Point(30, 100), cv::FONT_ITALIC, 1, cv::Scalar(255, 255, 255),
                        2);
            if (point_buffer.size() == 1) {
                cv::Rect rect(point_buffer[0], cv::Point(x_now, y_now));
                cv::rectangle(src, rect, cv::Scalar(128, 128, 128), 1);
            } else if (point_buffer.size() == 2) {
                cv::Rect rect(point_buffer[0], point_buffer[1]);
                cv::rectangle(src, rect, cv::Scalar(0, 255, 0), 1);
            }
            cv::line(src, cv::Point(0, y_now), cv::Point(width, y_now), cv::Scalar(0, 255, 255), 1);
            cv::line(src, cv::Point(x_now, 0), cv::Point(x_now, height), cv::Scalar(0, 255, 255), 1);


        }

        void mannul_mark::GetKeyAct() {
            return;
            while (main_program_run_flag) {
            }
        }

        mannul_mark::~mannul_mark() {
            main_program_run_flag = false;
        }

        void mannul_mark::SetFrameData(const DetectFrame &data) {
            frame_data_list.clear();
            for (auto &item : data.object()) {
                if (item.name() != "person") continue;
                cv::Rect2f rect_temp((item.x() - item.width() / 2) * width,
                                     (item.y() - item.height() / 2) * height,
                                     (item.width()) * width,
                                     (item.height()) * height);
                int count = 0;
                int index_it = -1;
                std::string name_it;
                for (auto &item2 : last_list) {
                    if (common::CalculateRectOverlapRatio(item2.track_data.bbox, rect_temp) > 0) {
                        count++;
                        index_it = item2.track_data.object_index;
                        name_it = item2.track_data.name;
                    }
                }
                if (count != 1)continue;
                frame_data_list.emplace_back(data_set(TrackData(rect_temp, name_it, index_it, item.probility()), true));
            }
        }

        std::string mannul_mark::GetStringFromType(const OBJECT_TYPE &in) {
            if (in == PERSON)return "person";
            else if (in == BICYCLE) return "bicycle";
            else if (in == MOTORBIKE)return "motorbike";
            else return "person";
        }

        std::string mannul_mark::GetStringFromState(const mannul_mark::MARK_STATUS &in) {
            if (in == INIT_FRAME)return "init frame";
            else if (in == NEW_BBOX) return "new bbox";
            else if (in == CHANGE_BBOX) return "change bbox";
            return "";
        }

        void mannul_mark::SaveToFile() {
            DetectFrame frame_data_tmp_proto;
            for (auto item : frame_data_list) {
                auto *iter = frame_data_tmp_proto.add_object();
                iter->set_name(item.track_data.name);
                iter->set_probility(101);
                iter->set_x(item.track_data.bbox.x);
                iter->set_width(item.track_data.bbox.width);
                iter->set_y(item.track_data.bbox.y);
                iter->set_height(item.track_data.bbox.height);
                iter->set_car_index(item.track_data.object_index);
            }
            std::string it_path(mannul_config.out_data_path());
            it_path += "/";
            it_path += std::to_string(count);
            it_path += ".bin";
            common::WriteProtoToBinaryFile(it_path, &frame_data_tmp_proto);
            mannul_temp_data.set_count_now(count);
            mannul_temp_data.set_trajectory_now(object_index);
            common::WriteProtoToBinaryFile(mannul_config.out_temp_path(), &mannul_temp_data);
        }

        void mannul_mark::init() {
            if (common::ReadProtoFromBinaryFile(mannul_config.out_temp_path(), &mannul_temp_data)) {
                cv::Mat mat_temp;
                for (int i = 0; i <= mannul_temp_data.count_now(); i++) {
                    video_capture >> mat_temp;
                    AddCount();
                }
                std::string it_path(mannul_config.out_data_path());
                it_path += "/";
                it_path += std::to_string(count - 1);
                it_path += ".bin";
                DetectFrame frame_temp;
                common::ReadProtoFromBinaryFile(it_path, &frame_temp);
                last_list.clear();
                for (auto &item : frame_temp.object()) {
                    cv::Rect2f rect_tmp(item.x(), item.y(), item.width(), item.height());
                    last_list.emplace_back(
                            data_set(TrackData(rect_tmp, item.name(), item.car_index(), item.probility()), true));
                }
                object_index = mannul_temp_data.trajectory_now();
                frame_data_list = last_list;
                cv::Mat l_canvas = mat_temp.clone();
                DrawRectangle(l_canvas);
                cv::imshow("last_image", l_canvas);
            }
        }


    }
}

MAIN(rs::vp::mannul_mark, "mannul_mark");
