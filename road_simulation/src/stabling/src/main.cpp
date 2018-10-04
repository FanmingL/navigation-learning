/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-10-03 15:16:26
*   Description : 
*
*===============================================================*/
#include "stabling.h"

int main(int argc, char **argv) {
    stabling stabling1;
    cv::Mat dst;
#ifdef WRITE_VIDEO
    std::string out_path = stabling1.car_filter1.base_path + "/data/stabling_out_video.mp4";
    std::remove((char *) out_path.c_str());
    cv::VideoWriter videoWriter(out_path, CV_FOURCC('D', 'I', 'V', 'X'), \
           30, cv::Size(960, 960));
#endif
    while (stabling1.run(dst)) {
#ifdef WRITE_VIDEO
        videoWriter << dst;
#endif
#ifdef SHOW_VIDEO
        cv::imshow("11", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif
    }
    return 0;
}
