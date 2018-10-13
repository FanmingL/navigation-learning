/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : track.cpp
*   Author      : FanmingL
*   Created date: 2018-10-12 13:21:50
*   Description : 
*
*===============================================================*/


#include "modules/track/select_track_algorithm.h"
#include "modules/track/track.h"
#include "track.h"


namespace rs{
    namespace vp{

        track::track(const std::string &name) : rs(name) {
            ReadConfig();
            ReadData();
            track_algorithm = common::AlgorithmFactory<BaseTrackAlgorithm>::CreateAlgorithm(track_config.algorithm_name());
            //video_capture.open(common::.in_video_path());
            video_capture.open(common::get_absolute_path(track_config.in_video_path()));
            int width = (int)video_capture.get(CV_CAP_PROP_FRAME_WIDTH);
            int height = (int)video_capture.get(CV_CAP_PROP_FRAME_HEIGHT);
            if (track_config.if_write_video())
            {
                std::string write_video_path = common::get_absolute_path(track_config.write_video_path());
                std::remove((char*)write_video_path.c_str());
                video_writer.open(write_video_path, CV_FOURCC('D', 'I', 'V', 'X'), 30, cv::Size(width, height));
            }
            counter = 0;
        }

        void track::Run() {
            cv::Mat src, dst;
            std::vector<TrackData> res;
            while (true)
            {
                video_capture >> src;
                if (src.empty())break;
                std::cout<<counter<<std::endl;
                track_algorithm->Track(src, dst, detect_data.frame(counter),res);
                auto iter_proto = detect_video.add_frame();
                for (auto &item : res)
                {
                    AddObject(item, iter_proto->add_object());
                }
                if (track_config.if_show_video())
                {
                    cv::imshow("track", dst);
                    auto key = cv::waitKey(1);
                    if (key == 'q')break;
                }
                if(track_config.if_write_video())
                {
                    video_writer << dst;
                }
                counter++;
            }
            common::WriteProtoToBinaryFile(track_config.out_data_path(), &detect_video);
        }

        void track::ReadConfig() {
            common::ReadProtoFromTextFile("modules/track/config/track.prototxt", &track_config);
        }

        void track::ReadData() {
           common::ReadProtoFromBinaryFile(track_config.data_path(), &detect_data);
        }

        void track::AddObject(const TrackData &data, DetectObject *object) {
            object->set_name(data.name);
            object->set_probility(data.probility);
            object->set_car_index(data.object_index);
            object->set_height(data.bbox.height);
            object->set_width(data.bbox.width);
            object->set_x(data.bbox.x);
            object->set_y(data.bbox.y);
        }


    }
}

MAIN(rs::vp::track, "track");
