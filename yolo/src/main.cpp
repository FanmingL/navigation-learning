
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

int main(int argc, char **argv)
{
    YOLO_DARKNET net;
    cv::Mat m = cv::imread(net.basePath + "/data/person.jpg");
    std::vector<YOLO_DARKNET::YOLO_OUT> res;
    net.yoloProcess(m, m, res, 0.5, 0.5, 0.4);
    for (auto &item : res)
        std::cout<<item<<std::endl;
    cv::imshow("111", m);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
