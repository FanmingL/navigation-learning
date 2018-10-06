/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : draw_area.h
*   Author      : FanmingL
*   Created date: 2018-10-06 13:32:00
*   Description : 
*
*===============================================================*/


#ifndef _DRAW_AREA_H
#define _DRAW_AREA_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <mutex>

class draw_area{
public:
    draw_area();

    cv::Mat first_image, canvas, mask, mask_cant_reach;

    void draw_position();

    void read_position();

    const std::string base_path = PATH;

    void mouse_cb(int event, int x, int y, int flags) ;

    static void onMouse(int event, int x, int y, int flag, void *user_data);
private:
    std::mutex init_image_mutex;

    int click_step;

    std::vector<cv::Point> click_buffer;

    std::vector<int> val_buffer;
};

#endif //DRAW_AREA_H
