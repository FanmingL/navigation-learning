/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-30 15:49:37
*   Description : 
*
*===============================================================*/
#include "line_finder.h"

int main(int argc, char **argv)
{
    line_finder line_finder1;
    cv::Mat dst;
    while (line_finder1.run(dst))
    {
#ifdef SHOW_VIDEO
    cv::imshow("111", dst);
    auto key = cv::waitKey(1);
    if (key == 'q')break;
#endif
    }

    return 0;
}
