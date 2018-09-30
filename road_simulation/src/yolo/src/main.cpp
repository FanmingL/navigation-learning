
/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-23 10:35:29
*   Description : 
*
*===============================================================*/
#include "YOLO_DARKNET.h"

int main(int argc, char **argv) {
    YOLO_DARKNET net;
#if 0
    std::ofstream outfile;
    outfile.open(net.basePath+"/data/yolo_out_data_image.txt");
    if (!outfile)std::cout<<"Error"<<std::endl;
    cv::Mat m = cv::imread(net.basePath + "/data/person.jpg");
    std::vector<YOLO_DARKNET::YOLO_OUT> res;
    for (auto &item : res)
        std::cout<<item<<std::endl;
    net.yoloProcess(m, m, res, 0.5, 0.5, 0.4);
    cv::imwrite(net.basePath+"/data/out.jpg", m);
    for (auto &item : res)
        outfile << item << std::endl<<std::endl;
/*    cv::imshow("111", m);
    cv::waitKey(0);
    cv::destroyAllWindows();*/
#else

    net.videoProcess("/data/in_video.mp4", "/data/yolo_out_video.mp4");
    //net.videoProcess("/data/DJI00004.MP4", "/data/RSD-004-13min-res.mp4");
#endif

    return 0;
}
