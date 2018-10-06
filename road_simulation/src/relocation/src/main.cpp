/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2018-10-04 18:48:49
*   Description : 
*
*===============================================================*/
#include "relocation.h"

void add_a_car(rs::object *car, relocation::car_data &data)
{
    car->set_frame_index(data.frame_count);
    car->set_object_index(data.index);

    auto point = new rs::point;
    point->set_x((float)(data.center_in_world.x));
    point->set_y((float)(data.center_in_world.y));
    car->set_allocated_world_center(point);

    /*point = new rs::point;
    point->set_x(0);
    point->set_y(0);
    car->set_allocated_world_velocity(point);
*/
    auto rect  = new rs::rect;
    rect->set_x((float)data.bbox_in_world.x);
    rect->set_y((float)data.bbox_in_world.y);
    rect->set_width((float)data.bbox_in_world.width);
    rect->set_height((float)data.bbox_in_world.height);
    car->set_allocated_world_position(rect);

    rect = new rs::rect;
    rect->set_x((float)data.bbox_in_image.x);
    rect->set_y((float)data.bbox_in_image.y);
    rect->set_height((float)data.bbox_in_image.height);
    rect->set_width((float)data.bbox_in_image.width);
    car->set_allocated_image_position(rect);
}

void add_a_frame(rs::frame *frame, std::vector<relocation::car_data> &data)
{
    /*std::sort(data.begin(), data.end(), [](relocation::car_data &t1,
            relocation::car_data &t2)->bool{
        return t1.index < t2.index;
    });*/
    std::reverse(data.begin(), data.end());
    frame->set_frame_index(data.back().frame_count);
    for (auto &item : data)
    {
        add_a_car(frame->add_objects(), item);
    }
}

void add_a_matrix(rs::homograph_matrix *frame, cv::Mat &homo)
{
    for (auto iter = homo.begin<float>();iter!= homo.end<float>();iter++)
    {
        frame->add_coefficient(*iter);
    }
}

int main(int argc, char **argv) {
    relocation relocation1;
    cv::Mat dst;
    std::vector<relocation::car_data> res;
    std::ofstream of;
    /*std::string data_out_path(relocation1.car_reader.base_path + "/data/relocation_out_data.txt");
    std::remove((char *) data_out_path.c_str());
    of.open(data_out_path);*/
    rs::video video_data;
    rs::all_homograph_matrix all_homograph_matrix;
    cv::Mat m_init(3,3,CV_32FC1, cv::Scalar(0));
    m_init.at<float>(0,0) = m_init.at<float>(1,1) = m_init.at<float>(2,2) = 1;
    add_a_matrix(all_homograph_matrix.add_single_homograph_matrix(),m_init);
#ifdef WRITE_VIDEO
    std::string video_out_path = relocation1.car_reader.base_path + "/data/relocation_out_video.mp4";
    std::remove((char *) video_out_path.c_str());
    cv::VideoWriter videoWriter(video_out_path, CV_FOURCC('D', 'I', 'V', 'X'), \
           30, cv::Size(960, 960));
#endif
    while (relocation1.run(dst, res)) {
#ifdef SHOW_VIDEO
        cv::imshow("111", dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif
#ifdef WRITE_VIDEO
        videoWriter << dst;
#endif
        add_a_frame(video_data.add_frames(),res);
        add_a_matrix(all_homograph_matrix.add_single_homograph_matrix(),relocation1.homography_to_first);
        /*for (auto &item : res) {
            of << item.frame_count << " " << item.index << " " << item.bbox_in_world.x << " " << item.bbox_in_world.y
               << " "
               << item.bbox_in_world.width << " " << item.bbox_in_world.height << std::endl;
        }*/
        std::cout << car_filter::car_data::frame_index << std::endl;
    }
    {
        std::ofstream off(relocation1.car_reader.base_path + "/data/all_car.proto.data", std::ios::trunc | std::ios::binary);
        video_data.SerializePartialToOstream(&off);
    }
    {
        std::ofstream off(relocation1.car_reader.base_path + "/data/all_matrix.proto.data", std::ios::trunc | std::ios::binary);
        all_homograph_matrix.SerializePartialToOstream(&off);
    }
    return 0;
}

