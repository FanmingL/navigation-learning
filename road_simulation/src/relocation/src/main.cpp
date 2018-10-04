/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-10-04 18:48:49
*   Description : 
*
*===============================================================*/
#include "relocation.h"
int main(int argc, char **argv)
{
    relocation relocation1;
    cv::Mat dst;
    std::vector<relocation::car_data> res;
    std::ofstream of;
    std::string data_out_path(relocation1.car_reader.base_path +  "/data/relocation_out_data.txt");
    std::remove((char*)data_out_path.c_str());
    of.open(data_out_path);
#ifdef WRITE_VIDEO
    std::string video_out_path = relocation1.car_reader.base_path +"/data/relocation_out_video.mp4";
    std::remove((char*)video_out_path.c_str());
    cv::VideoWriter videoWriter(video_out_path,CV_FOURCC('D', 'I', 'V', 'X'),\
           30, cv::Size(960, 960));
#endif
    while (relocation1.run(dst, res))
    {
#ifdef SHOW_VIDEO
        cv::imshow("111", dst);
        auto key= cv::waitKey(1);
        if (key == 'q')break;
#endif
#ifdef WRITE_VIDEO
    videoWriter << dst;
#endif
        for (auto &item : res)
        {
            of << item.frame_count << " " <<item.index << " "<<item.bbox_in_world.x << " "<<item.bbox_in_world.y<<" "
            <<item.bbox_in_world.width <<" "<<item.bbox_in_world.height<<std::endl;
        }
        std::cout<<car_filter::car_data::frame_index<<std::endl;
    }
    return 0;
}

