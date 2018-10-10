/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : tracker.h
*   Author      : FanmingL
*   Created date: 2018-10-11 01:03:17
*   Description : 
*
*===============================================================*/


#ifndef _TRACKER_H
#define _TRACKER_H
#include "common/io.h"
#include "common/main_interface.h"
#include "common/rs.h"
#include "tracker.pb.h"
#include <string>


namespace rs{
    namespace vp{
        class tracker : public common::rs{
        public:
            explicit tracker(const std::string &name);
            ~tracker() override = default;
            void Run() override ;
            bool ReadConfig();
        private:
            yolo_video video_proto_data;
        };
    }
}

#endif //TRACKER_H
