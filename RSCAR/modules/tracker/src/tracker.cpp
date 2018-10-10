/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : tracker.cpp
*   Author      : FanmingL
*   Created date: 2018-10-11 01:03:11
*   Description : 
*
*===============================================================*/


#include "tracker.h"


namespace rs{
    namespace vp{

        tracker::tracker(const std::string &name) : rs(name) {
            if (ReadConfig())
                std::cout<<"Error While Read"<<std::endl;
            std::cout<<video_proto_data.DebugString()<<std::endl;
        }

        void tracker::Run() {

        }

        bool tracker::ReadConfig() {
            return common::ReadProtoFromBinaryFile("data/yolo_out_data.proto.b",
                    &video_proto_data);
        }
    }
}


MAIN(rs::vp::tracker, "tracker");