/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-09-07 15:13:15
*   Description : 
*
*===============================================================*/


#include "main.h"
int main(int argc, char **argv)
{
    const float measureCov = 100, controlCov = 4;
    MATRIX_TYPE measureMatrix = MATRIX_I * measureCov,
                controlMatrix = MATRIX_I * controlCov;
    KalmanFilter kf(measureMatrix, controlMatrix);
    const float omega = 3.1415f/5, radius = 100;
    auto img = cv::Mat(640, 480, CV_8UC3);
    cv::circle(img, cv::Point(img.cols/2, img.rows/2), (int)radius, cv::Scalar(255, 255, 255), -1);
    int key;
    float angle_now;
    float period = 30.f;
    float lineVelocity = omega * radius;
    float radius_mini = 10;
    cv::RNG rng(time(NULL));
    for (;;)
    {
        cv::Mat img_draw;
        img.copyTo(img_draw);
        float vx = -lineVelocity * sinf(angle_now), vy = lineVelocity * cosf(angle_now);
        angle_now += omega * period / 1000;
        float x = cosf(angle_now) * radius, y= sinf(angle_now) * radius;

        float control_rand[2], measure_rand[2];
        control_rand[0] = (float)rng.gaussian(sqrtf(controlCov)), control_rand[1] = (float)rng.gaussian(sqrtf(controlCov));
        measure_rand[0] = (float)rng.gaussian(sqrtf(measureCov)), measure_rand[1] = (float)rng.gaussian(sqrtf(measureCov));
        VECTOR_TYPE measure, res, control;
        measure(0, 0) = measure_rand[0] + x;
        measure(1, 0) = measure_rand[1] + y;
        control(0, 0) = control_rand[0] + vx;
        control(1, 0) = control_rand[1] + vy;
        control = control * period / 1000;
        kf.correct(measure, control, res);
        cv::circle(img_draw, cv::Point(measure_rand[0] + x + img.cols/2, measure_rand[1] + y + img.rows/2),\
                   radius_mini+10, cv::Scalar(255, 255, 0), 2);
        cv::circle(img_draw, cv::Point(x + img.cols/2, y + img.rows/2), radius_mini, cv::Scalar(255, 0, 0), -1);
        cv::circle(img_draw, cv::Point(res(0,0) + img.cols/2, res(1, 0) + img.rows/2),
                   radius_mini, cv::Scalar(0, 0, 255), 2);
        cv::imshow("test", img_draw);
        key = cv::waitKey(period);
        if (key == 'q')break;
    }


    cv::destroyWindow("test");




    return 0;
}
