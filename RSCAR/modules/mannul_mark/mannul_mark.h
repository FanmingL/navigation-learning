/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : mannul_mark.h
*   Author      : FanmingL
*   Created date: 2018-10-24 22:32:42
*   Description : 
*
*===============================================================*/


#ifndef _MANNUL_MARK_H
#define _MANNUL_MARK_H

#include "common/rs.h"
#include "common/main_interface.h"
#include "common/image_util.h"
#include "common/io.h"

#include <mutex>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <tuple>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/tracking.hpp"

#include "modules/detect/detect.pb.h"
#include "modules/mannul_mark/mannul_mark.pb.h"

#include "modules/track/base_track_algorithm.h"
namespace rs{
    namespace vp{
        class mannul_mark : public common::rs{
        public:
            enum MARK_STATUS{
                INIT_FRAME = 0,
                CHANGE_BBOX ,
                NEW_BBOX
            };
            enum OBJECT_TYPE{
                PERSON = 0,
                BICYCLE ,
                MOTORBIKE
            };
            class data_set{
            public:
                data_set(const TrackData &_track_data, bool _index_flag = false) : track_data(_track_data),
                index_flag(_index_flag){}
                TrackData track_data;
                bool index_flag;
            };
            typedef std::tuple<TrackData, bool> data_tuple;
            explicit mannul_mark(const std::string & name);
            ~mannul_mark() override;
            void Run() override ;
            void ReadConfig();
            void ReadData();
            static void onMouse(int event, int x, int y, int flag, void *user_data);
            void MouseCb(int event, int x, int y, int flags);
            void AddCount();
            void DrawRectangle(cv::Mat &src);
            void GetKeyAct();
            void SetFrameData(const DetectFrame &data);
            std::string GetStringFromType(const OBJECT_TYPE & in);
            std::string GetStringFromState(const MARK_STATUS & in);
            void SaveToFile();
        private:
            DetectVideo detect_video;
            MannulConfig mannul_config;
            cv::VideoCapture video_capture;
            int count;
            int width, height;
            cv::Rect max_rect;
            const std::string canvas_window_name = std::string("canvas");
            std::list<data_set> frame_data_list, last_list;
            std::mutex image_step_mutex, show_image_mutex;
            bool step_flag;
            bool main_program_run_flag;
            cv::Mat frame, canvas;
            int key;
            MARK_STATUS mark_state;
            std::vector<cv::Point> point_buffer;
            int object_index;
            OBJECT_TYPE object_type;
            int x_now, y_now;
        };
    }
}

#endif //MANNUL_MARK_H
