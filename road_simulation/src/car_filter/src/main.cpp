/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:39
*   Description :
*
*===============================================================*/


#include "car_filter.h"

int main(int argc, char **argv) {
    car_filter car_filter1(std::string("/data/filter_out_data.txt"));
    std::string out_data_2 = car_filter1.base_path+"/data/filter_out_data_normal.txt";
    std::remove((char*)out_data_2.c_str());
    std::ofstream of;
    of.open(out_data_2);
#ifdef WRITE_VIDEO
    std::string video_out_path = car_filter1.base_path +"/data/filter_out_video.mp4";
    std::remove((char*)video_out_path.c_str());
    cv::VideoWriter videoWriter(video_out_path,CV_FOURCC('D', 'I', 'V', 'X'),\
           30, cv::Size(car_filter1.frame_width, car_filter1.frame_height));
#endif

    cv::Mat dst;
    std::vector<std::pair<int, cv::Rect2d> > res;
    int count = 0;
    while (car_filter1.run(dst, res)) {
    for (auto &item : res)
            of << car_filter::car_data::frame_index << " " << item.first << " " << item.second.x << " "\
        << item.second.y << " " << item.second.width << " " << item.second.height << std::endl;
        std::cout << car_filter::car_data::frame_index << std::endl;
#ifdef SHOW_VIDEO
        cv::imshow("11", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif
#ifdef WRITE_VIDEO
        if (count%3==0)
        videoWriter << dst;
#endif
        std::cout << count++ << std::endl;
    }

    return 0;
}

