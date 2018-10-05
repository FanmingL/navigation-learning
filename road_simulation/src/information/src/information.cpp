/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : information.cpp
*   Author      : FanmingL
*   Created date: 2018-10-05 13:51:37
*   Description : 
*
*===============================================================*/


#include <src/information/include/information.h>

#include "information.h"

information::information(float _distance_thrashold) : distance_threshold(_distance_thrashold){
}


void information::run() {
#if 1
    read_from_binary(&video_data, base_path+ "/data/all_car.proto.data");
    std::unordered_map<int, std::shared_ptr<filter_algorithm_base> > filter_map;
    std::unordered_map<int, rs::trajectory*> trajectory_map;
    int max_index_now = -1;
    rs::object_pro object_pro;
    for (auto &frame: video_data.frames())
    {
        std::cout<<frame.frame_index()<<std::endl;
        std::unordered_map<int, cv::Point2d> position_after_filter;
        std::unordered_map<int, cv::Point2d> velocity;
        std::unordered_map<int, int> index_to_map;
        int j = 0;
        for (; j < frame.objects().size(); j++)
        {
            auto & item = frame.objects(j);
            if (item.object_index() > max_index_now)break;
            if (filter_map.count(item.object_index()))
            {
                cv::Point2d point_tmp(item.world_center().x(), item.world_center().y());
                velocity[item.object_index()] = filter_map[item.object_index()]->run(point_tmp, point_tmp) * 30.0;
                position_after_filter[item.object_index()] = point_tmp;
                index_to_map[item.object_index()] = j;
            }
        }
        /// gap is the first who joins the trajectory
        int gap = j;

        std::vector<int> index_to_delete;
        for (auto &item : filter_map)
        {
            if (!position_after_filter.count(item.first))
            {
                index_to_delete.push_back(item.first);
            }
        }

        for (auto &item : index_to_delete)
        {
            filter_map.erase(item);
            trajectory_map.erase(item);
        }

        for (; j < frame.objects().size(); j++)
        {
            auto & item = frame.objects(j);
            filter_map[item.object_index()] = std::make_shared<move_mean>(cv::Point2d(item.world_center().x(),
                    item.world_center().y()), 4);
            trajectory_map[item.object_index()] = all_trajectory.add_trajectories();
            max_index_now = item.object_index();
            position_after_filter[item.object_index()] = cv::Point2d(item.world_center().x(),
                    item.world_center().y());
            index_to_map[item.object_index()] = j;
        }
        for (auto &item : position_after_filter)
        {
            auto iter = trajectory_map[item.first]->add_objects();
            auto object_iter = new rs::object(frame.objects(index_to_map[item.first]));

            auto point_iter = new rs::point();
            point_iter->set_x((float)position_after_filter[item.first].x);
            point_iter->set_y((float)position_after_filter[item.first].y);
            object_iter->set_allocated_world_center(point_iter);
            iter->set_allocated_it(object_iter);
            point_iter = new rs::point();
            double head_angle = 0;
            if (velocity.count(item.first))
            {
                point_iter->set_x((float)velocity[item.first].x);
                point_iter->set_y((float)velocity[item.first].y);
                head_angle = atan2(velocity[item.first].y, velocity[item.first].x);
            } else{
                point_iter->set_x(0);
                point_iter->set_y(0);
            }
            iter->set_allocated_velocity(point_iter);
            for (auto &item2 : position_after_filter) {
                cv::Point2d point_2_tmp(iter->it().world_center().x(), iter->it().world_center().y());
                if (point_2_tmp.x == item2.second.x || point_2_tmp.y == item2.second.y)continue;
                auto point_3_tmp = item2.second - point_2_tmp;
                auto distance = cv::norm(point_3_tmp);
                if (distance > distance_threshold || distance < 1)continue;
                auto iter_surround = iter->add_obj();
                iter_surround->set_distance_to_center((float) distance);
                auto angle_absolute = atan2(point_3_tmp.y, point_3_tmp.x);
                iter_surround->set_angle_to_center((float) ((angle_absolute - head_angle) / CV_PI * 180));
                iter_surround->set_index(item2.first);
                iter_surround->set_surround_type(rs::surround_type::BUS);
                auto rect_iter = new rs::rect();
                rect_iter->set_x(frame.objects(index_to_map[item2.first]).world_position().x());
                rect_iter->set_y(frame.objects(index_to_map[item2.first]).world_position().y());
                rect_iter->set_width(frame.objects(index_to_map[item2.first]).world_position().width());
                rect_iter->set_height(frame.objects(index_to_map[item2.first]).world_position().height());
                auto obj_center_iter = new rs::point();
                obj_center_iter->set_x((float)item2.second.x);
                obj_center_iter->set_y((float)item2.second.y);
                iter_surround->set_allocated_obj_position(obj_center_iter);
                iter_surround->set_allocated_obj_rect(rect_iter);
            }
        }
    }
    write_to_binary(&all_trajectory, base_path+"/data/all_trajectories.proto.data");
#else
    read_from_binary(&all_trajectory, base_path + "/data/all_trajectories.proto.data");
    print_one_trajectory(all_trajectory.trajectories(0));
#endif
}



void information::print_one_trajectory(const rs::trajectory &trajectory) {
    std::cout<<"trajectory index: "<<trajectory.objects(0).it().object_index()<<std::endl;
    std::cout<<"target position:"<<trajectory.objects(trajectory.objects().size()-1).it().world_center().x()<<", "
    <<trajectory.objects(trajectory.objects().size()-1).it().world_center().y()<<std::endl;
    std::cout<<"remain time length(s): "<<trajectory.objects().size()*1/30.<<std::endl<<std::endl;
    for (auto &item : trajectory.objects())
    {
        std::cout<<"frame index: "<<item.it().frame_index()<<std::endl;
        std::cout<<"current position: "<<item.it().world_center().x()<<", "<<item.it().world_center().y()<<std::endl;
        std::cout<<"obstacle: "<<std::endl;
        for (auto &item2 : item.obj())
        {
            std::cout<<"distance: "<<item2.distance_to_center()<<", relative angle: "<<item2.angle_to_center()
            <<", object index: "<<item2.index()<<", object position: "<<item2.obj_position().x()
            <<", "<<item2.obj_position().y()<<", object type: "<<item2.surround_type()<<std::endl;
        }
    }
}




