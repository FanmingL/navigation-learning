/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : car_filter.cpp
*   Author      : FanmingL
*   Created date: 2018-09-28 15:39:44
*   Description : 
*
*===============================================================*/


#include "car_filter.h"

int car_filter::car_data::frame_now = -1;

car_filter::car_filter() {
    ff.open(base_path + "/data/in_data2.txt");
    assert(ff.is_open());
}


