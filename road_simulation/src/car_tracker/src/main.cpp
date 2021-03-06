/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-26 22:13:21
*   Description : 
*
*===============================================================*/

#include "car_tracker.h"

int main(int argc, char **argv) {
    car_tracker car_tracker1(3.f, 10.0f, 60, 3, 450, 0.6, 0.35);
    cv::Mat dst;
    std::vector<std::pair<int, cv::Rect2d> > res;

    std::string out_path = car_tracker1.base_path + "/data/out_video.mp4";
    std::string out_txt_path = car_tracker1.base_path + "/data/tracker_out_data.txt";
    std::remove((char *) out_path.c_str());
    std::remove((char *) out_txt_path.c_str());
    std::ofstream of;
    of.open(out_txt_path);
    assert(of.is_open());
#ifdef WRITE_VIDEO
    cv::VideoWriter videoWriter(out_path, CV_FOURCC('D', 'I', 'V', 'X'), \
           30, cv::Size(car_tracker1.frame_width, car_tracker1.frame_height));
#endif
    while (car_tracker1.run(dst, res)) {
        /*for (auto &item : res)
            std::cout<<item.first<<std::endl;*/
        std::sort(res.begin(), res.end(), [](std::pair<int, cv::Rect2d> x,
                                             std::pair<int, cv::Rect2d> y) -> bool { return x.first < y.first; });
        for (auto &item : res)
            of << car_tracker::yolo_car::frame_index << " " << item.first << " " << item.second.x << " "\
 << item.second.y << " " << item.second.width << " " << item.second.height << std::endl;
        std::cout << car_tracker::yolo_car::frame_index << std::endl;
#ifdef WRITE_VIDEO
        videoWriter << dst;
#endif
#ifdef SHOW_VIDEO
        cv::imshow("111", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif
    }
    of.close();


    return 0;
}
