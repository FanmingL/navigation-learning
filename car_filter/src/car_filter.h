/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_filter.h
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:51
*   Description : 
*
*===============================================================*/


#ifndef _CAR_FILTER_H
#define _CAR_FILTER_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/tracking.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <unordered_set>
#include <mutex>
#include <algorithm>
#include <thread>

class car_filter {
public:
    class car_data {
        car_data();

        ~car_data() = default;

        int index;
        cv::Rect2d bbox;
        int it_frame;
        static int frame_now;
    };

    car_filter();

    ~car_filter() = default;


    const std::string base_path = std::string(PATH);
private:
    std::ifstream ff;

};

#endif //CAR_FILTER_H
