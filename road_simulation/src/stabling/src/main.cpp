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

int main(int argc, char**argv)
{
    stabling stabling1;
    cv::Mat dst;
    while (stabling1.run(dst))
    {
        cv::imshow("11", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
    }
    return 0;
}
