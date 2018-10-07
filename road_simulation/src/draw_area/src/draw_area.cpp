/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : drwa_area.cpp
*   Author      : FanmingL
*   Created date: 2018-10-06 13:32:31
*   Description : 
*
*===============================================================*/



#include <src/draw_area/include/draw_area.h>

#include "draw_area.h"

draw_area::draw_area() : mask(960, 960, CV_8UC1, cv::Scalar(255)), click_step(0) {
    first_image = cv::imread(base_path + "/data/first_frame.jpg");
    for (int i = 0; i < 10; i++)
        val_buffer.push_back(i*20);
    mask = cv::imread(base_path + "/data/cant_touch_place_mask.png", cv::IMREAD_GRAYSCALE);

    draw_position();
    //read_position();

}
void draw_area::mouse_cb(int event, int x, int y, int flags) {
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            std::cout << x << ", " << y << std::endl;
            click_buffer.emplace_back(cv::Point(x, y));
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        default:
            break;
    }

}

void draw_area::onMouse(int event, int x, int y, int flag, void *user_data) {
    auto *temp = reinterpret_cast<draw_area *>(user_data);
    temp->mouse_cb(event, x, y, flag);
}

void draw_area::draw_position() {
    cv::imshow("first_image", first_image);
    cv::setMouseCallback("first_image", draw_area::onMouse, this);
    int fill_val = 0;
    while (true)
    {
        canvas = cv::Scalar(0,0,0);
        first_image.copyTo(canvas, mask > 230);
        for (auto &item : click_buffer)
        {
            cv::circle(canvas, item, 2, cv::Scalar(0,0,255),-1);
        }
        for (int i = 1; i < click_buffer.size(); i++)
        {

            cv::line(canvas,click_buffer[i-1], click_buffer[i],cv::Scalar(0,255,0),2);
        }
        cv::putText(canvas, std::to_string(fill_val), cv::Point(10,10),CV_FONT_BLACK,0.5, cv::Scalar(0, 0, 255));
        cv::imshow("first_image", canvas);
        auto key = cv::waitKey(30);
        if (key == 'q')break;
        switch (key){
            case ' ':
                cv::fillPoly(mask, std::vector<std::vector<cv::Point> >(1, click_buffer), cv::Scalar(fill_val));
                click_buffer.clear();
                break;
            case 'a':
                if (!click_buffer.empty())
                    click_buffer.pop_back();
                break;
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                fill_val = val_buffer[(key - '0')];
                break;
            case 'y':
                fill_val = 200;
                break;
            default:
                break;
        }
    }
    std::vector<int> parameters;
    parameters.push_back(CV_IMWRITE_PNG_COMPRESSION);
    parameters.push_back(0);
    cv::imwrite(base_path + "/data/cant_touch_place_mask.png", mask, parameters);
}

void draw_area::read_position() {
    mask_cant_reach = cv::imread(base_path + "/data/cant_touch_place_mask.png", cv::IMREAD_GRAYSCALE);

    cv::imshow("111",mask_cant_reach);
    cv::waitKey();
}
