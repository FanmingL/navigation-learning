/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : filter_tracker.cpp
*   Author      : FanmingL
*   Created date: 2018-10-27 12:16:50
*   Description : 
*
*===============================================================*/


#include "filter_tracker.h"


namespace rs{
    namespace vp{

        tracker_filter::tracker_filter(const std::string &name) : rs(name),count(0) {
           ReadConfig();
           ReadData();
           filter_tracker_config.PrintDebugString();
           video_capture.open(common::GetAbsolutePath(filter_tracker_config.in_video_path()));
           //mask_capture.open(common::GetAbsolutePath(filter_tracker_config.mask_video_path()));
           if (filter_tracker_config.if_write_video()){
               std::remove((char*)(common::GetAbsolutePath(filter_tracker_config.out_video_path()).c_str()));
               video_writer.open(common::GetAbsolutePath(filter_tracker_config.out_video_path()),
                       CV_FOURCC('D','I','V', 'X'),30,
                       cv::Size((int)video_capture.get(cv::CAP_PROP_FRAME_WIDTH),
                               (int)video_capture.get(cv::CAP_PROP_FRAME_HEIGHT)));
           }
           max_rect = cv::Rect(0,0,(int)video_capture.get(cv::CAP_PROP_FRAME_WIDTH),
                               (int)video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));
           max_rect_2f = max_rect;
           InitColorMap();
           InitLength();
           InitAreaRange();
        }

        void tracker_filter::Run() {
            cv::Mat canvas, mask_frame_one_channel;
            while (true){
                if (filter_tracker_config.if_write_video() || filter_tracker_config.if_show_video()){
                    video_capture >> frame;
                    //mask_capture >> frame_mask;
                    //if (frame.empty() || frame_mask.empty())break;
                    if (frame.empty())break;
                }
                if (count >=  in_video_data.frame_size())break;

                //common::GetOneChannel(frame_mask, mask_frame_one_channel, 1);

                //NewData(in_video_data.frame(count),out_video_data.add_frame(), raw_data.frame(count));
                NewData(in_video_data.frame(count), out_video_data.add_frame(), in_video_data.frame(count));
                if (filter_tracker_config.if_write_video() || filter_tracker_config.if_show_video()){
                    canvas = frame.clone();
                    DrawRect(canvas, out_video_data.frame(count));
                }

                if (filter_tracker_config.if_show_video()){
                    //cv::imshow("mask",mask_frame_one_channel);
                    cv::imshow("tracker_filter", canvas);
                    if (cv::waitKey(1) == 'q')break;
                }
                if (filter_tracker_config.if_write_video()){
                    video_writer << canvas;
                }
                AddCount();
            }
            DetectVideo out_video_data_new;
            for (auto &item : out_video_data.frame()){
                auto *iter1 = out_video_data_new.add_frame();
                for (auto &item2 : item.object()){
                    auto *iter2 = iter1->add_object();
                    iter2->set_x(item2.x());
                    iter2->set_y(item2.y());
                    iter2->set_width(item2.width());
                    iter2->set_height(item2.height());
                    iter2->set_probility(item2.probility());
                    iter2->set_car_index(item2.car_index());
                    if (bicycle_count.count(item2.car_index()) && bicycle_count[item2.car_index()] > filter_tracker_config.shift_type_count()){
                        iter2->set_name("bicycle");
                    }else if(motor_count.count(item2.car_index()) && motor_count[item2.car_index()] > filter_tracker_config.shift_type_count()){
                        iter2->set_name("motorbike");
                    }else{
                        iter2->set_name(item2.name());
                    }
                }
            }
            common::WriteProtoToBinaryFile(filter_tracker_config.out_data_path(), &out_video_data_new);
        }

        void tracker_filter::ReadConfig() {
            common::ReadProtoFromTextFile("modules/filter_tracker/config/filter_tracker.prototxt", &filter_tracker_config);
        }

        void tracker_filter::AddCount() {
            std::cout<< count ++<<std::endl;

        }

        void tracker_filter::ReadData() {
            common::ReadProtoFromBinaryFile(filter_tracker_config.in_data_path(), &in_video_data);
            common::ReadProtoFromBinaryFile(filter_tracker_config.raw_data_path(), &raw_data);
        }

        void tracker_filter::DrawRect(cv::Mat &src, const DetectFrame &data) {
            for (auto &item : data.object()){
                cv::Rect2f rect_tmp(item.x(), item.y(), item.width(), item.height());
                rect_tmp = rect_tmp & max_rect_2f;
                cv::rectangle(src, rect_tmp,color_map[item.name()],2);
                cv::putText(src, std::to_string(item.car_index()), rect_tmp.tl(),cv::FONT_ITALIC,0.8,cv::Scalar(0,0,255),2);
            }
        }

        void tracker_filter::InitColorMap() {
            color_map["person"] = cv::Scalar(255,0,0);
            color_map["car"] = cv::Scalar(255,255,0);
            color_map["bicycle"] = cv::Scalar(255,0,255);
            color_map["motorcycle"] = cv::Scalar(0,255,0);
            color_map["motorbike"] = cv::Scalar(0,0,255);
        }

        void tracker_filter::NewData(const DetectFrame &in_data, DetectFrame *out_data, const DetectFrame &raw_data) {
            for (auto &item: in_data.object()){
                cv::Rect2f rect_temp(item.x(), item.y(), item.width(), item.height());
                if (item.car_index() == -1){
                    auto *iter = out_data->add_object();
                    iter->set_name(item.name());
                    iter->set_probility(100);
                    iter->set_car_index(item.car_index());
                    iter->set_x(item.x());
                    iter->set_y(item.y());
                    iter->set_width(item.width());
                    iter->set_height(item.height());
                    continue;
                }
                if (length_map[item.car_index()] < 10)continue;
                if (area_low.count(item.name()) && rect_temp.area() < area_low[item.name()])continue;
                if (area_high.count(item.name()) && rect_temp.area() > area_high[item.name()]) continue;
                cv::Point2f p_tmp =  cv::Point2f(item.width(),item.height());
                cv::Point2f center = cv::Point2f(item.x() + item.width()/2, item.y()+ item.height()/2);
                if (size_filter_map.count(item.car_index())){
                    p_tmp = size_filter_map[item.car_index()]->Run(p_tmp);
                    center = pos_filter_map[item.car_index()]->Run(center);
                }else{
                    size_filter_map[item.car_index()] = std::make_shared<common::mean_filter<cv::Point2f> > (
                           p_tmp ,filter_tracker_config.size_mean_filter_length());
                    pos_filter_map[item.car_index()] = std::make_shared<common::mean_filter<cv::Point2f> > (
                            center ,filter_tracker_config.position_mean_filter_length());
                }
                /*if (item.probility() == 0)
                    std::cout<<length_map[item.car_index()]<<std::endl;*/
                //if(item.name()=="car")goto gap_it;
                if(true)goto gap_it;
                for (auto &item2 : raw_data.object()){
                    cv::Rect2f rect2(item2.x(),item2.y(), item2.width(), item2.height());
                    if (item2.name() == "motorcycle"){
                        if (common::CalculateRectOverlapRatio(rect2, rect_temp, common::AND_MIN) > filter_tracker_config.shift_overlap()){
                            motor_count[item.car_index()]++;
                        }
                    } else if(item2.name() == "bicycle"){
                        if (common::CalculateRectOverlapRatio(rect2, rect_temp, common::AND_MIN) > filter_tracker_config.shift_overlap()){
                            bicycle_count[item.car_index()]++;
                        }
                    }
                }
                gap_it:;
                auto *iter = out_data->add_object();
                if (bicycle_count.count(item.car_index()) && bicycle_count[item.car_index()] > filter_tracker_config.shift_type_count()){
                    iter->set_name("bicycle");
                }else if(motor_count.count(item.car_index()) && motor_count[item.car_index()] > filter_tracker_config.shift_type_count()){
                    iter->set_name("motorcycle");
                }else{
                    iter->set_name(item.name());
                }
                iter->set_probility(100);
                iter->set_car_index(item.car_index());
                iter->set_x(center.x - p_tmp.x/2);
                iter->set_y(center.y - p_tmp.y/2);
                iter->set_width(p_tmp.x);
                iter->set_height(p_tmp.y);
            }
        }

        void tracker_filter::InitLength() {
            for (auto &item : in_video_data.frame()){
                for (auto &item2 : item.object()){
                    if (item2.car_index() != -1)
                        length_map[item2.car_index()]++;
                }
            }
        }

        void tracker_filter::InitAreaRange() {
            area_low["car"] = filter_tracker_config.min_car_area();
        }

    }
}

MAIN(rs::vp::tracker_filter, "tracker_filter");