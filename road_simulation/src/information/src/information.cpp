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

information::information(float _distance_thrashold) : distance_threshold(_distance_thrashold)
,base_homograph(3,3,CV_32FC1,cv::Scalar(0))
{
    //read_from_binary(&all_matrix, base_path + "/data/all_matrix.proto.data");
    read_from_binary(&base_homograph_proto, base_path + "/data/homography_base.proto.data");
    first_image = cv::imread(base_path + "/data/first_image.png");
    mask = cv::imread(base_path + "/data/cant_touch_place_mask.png", cv::IMREAD_GRAYSCALE);
    int _i = 0;
    for (auto iter = base_homograph.begin<float>(); iter != base_homograph.end<float>();iter++,_i++)
        *iter = base_homograph_proto.coefficient(_i);
}


void information::run() {
#if 1
    read_from_binary(&video_data, base_path+ "/data/all_car.proto.data");
    //video_data.frames(2).PrintDebugString();
    float distance_add_step = 0.5f;
    std::unordered_map<int, std::shared_ptr<filter_algorithm_base> > filter_map;
    std::unordered_map<int, rs::trajectory*> trajectory_map;
    int max_index_now = -1;
    rs::object_pro object_pro;
    for (auto &frame :video_data.frames())
    {
        cv::Mat frame_mask = mask.clone();

        std::cout<<frame.frame_index()<<std::endl;
        if (frame.frame_index() > 13000)break;
        if (frame.frame_index() % 100 == 0){
            //std::cout<<all_trajectory.trajectories(all_trajectory.trajectories_size()/2).objects_size()<<std::endl;
            //std::ofstream offfff(base_path + "/data/dubug.txt");
            //offfff<< all_trajectory.trajectories(all_trajectory.trajectories_size()/2).DebugString();
            //std::cout<<all_trajectory.trajectories(all_trajectory.trajectories_size()/2).objects(0).obj_size()<<std::endl;
            //all_trajectory.trajectories(all_trajectory.trajectories_size()/2).PrintDebugString();
        }
        std::unordered_map<int, cv::Point2d> position_after_filter;
        std::unordered_map<int, cv::Point2d> velocity;
        std::unordered_map<int, int> index_to_map;
        int last = -1;
        std::vector<std::pair<int, int> > object_index_sort;
        for(int j = 0; j  < frame.objects_size(); j++)
        {
            object_index_sort.emplace_back(std::pair<int, int>(frame.objects(j).object_index(), j));
        }
        std::sort(object_index_sort.begin(),object_index_sort.end());
        int j = 0;

        for (; j < frame.objects().size(); j++)
        {

            auto & item = frame.objects(object_index_sort[j].second);
            if (item.object_index() > max_index_now)break;
            if (filter_map.count(item.object_index()))
            {
                cv::Point2d point_tmp(item.world_center().x(), item.world_center().y());
                velocity[item.object_index()] = filter_map[item.object_index()]->run(point_tmp, point_tmp) * 30.0;
                position_after_filter[item.object_index()] = point_tmp;
                //std::cout<<velocity[item.object_index()]<<std::endl;
                index_to_map[item.object_index()] = object_index_sort[j].second;
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
        //std::cout<<"delete size: "<<index_to_delete.size()<<std::endl;
        //std::cout<<"remain size: "<<trajectory_map.size()<<std::endl;

        for (; j < frame.objects().size(); j++)
        {
            auto & item = frame.objects(object_index_sort[j].second);

            cv::Point image_center = cv::Point((int)(item.image_position().x() + item.image_position().width()/2),
                                               (int)(item.image_position().y() + item.image_position().height()/2));
            //cv::Point image_tl = cv::Point((int)item.image_position().x(),(int)item.image_position().y());
            //cv::Point image_br = cv::Point((int)(item.image_position().x() + item.image_position().width()),
            //                                   (int)(item.image_position().y() + item.image_position().height()));
            if (!(cv::Rect(0,0,960,960).contains(image_center) &&
            //cv::Rect(0,0,960,960).contains(image_br) &&
            //cv::Rect(0,0,960,960).contains(image_tl) &&
            mask.at<uchar>(image_center) != 20 * ROAD_TYPE::AREA_DONT_CONSIDER &&
            mask.at<uchar>(image_center) != 20 * ROAD_TYPE::ROAD_CANOT_REACH)
            )
                continue;
            max_index_now = std::max(item.object_index(), max_index_now);
            filter_map[item.object_index()] = std::make_shared<move_mean>(cv::Point2d(item.world_center().x(),
                    item.world_center().y()), 4);
            trajectory_map[item.object_index()] = all_trajectory.add_trajectories();
            position_after_filter[item.object_index()] = cv::Point2d(item.world_center().x(),
                    item.world_center().y());
            index_to_map[item.object_index()] = object_index_sort[j].second;
        }
        for (auto &item : position_after_filter)
        {
            int car_index = index_to_map[item.first];
            cv::Rect rect_fill_color_tmp((int)frame.objects(car_index).image_position().x(),
                                (int)frame.objects(car_index).image_position().y(),
                                (int)frame.objects(car_index).image_position().width(),
                                (int)frame.objects(car_index).image_position().height());
            frame_mask(rect_fill_color_tmp & cv::Rect(0, 0, 960, 960)) = cv::Scalar(220);
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
                //std::cout<<head_angle<<std::endl;
            } else{
                point_iter->set_x(-1000);
                point_iter->set_y(-1000);
            }
            iter->set_allocated_velocity(point_iter);
            cv::Rect bound_rect(0,0,960,960);
                int it_index = index_to_map[item.first];
                cv::Point it_point((int)frame.objects(it_index).image_position().x(),
                        (int)frame.objects(it_index).image_position().y());
                cv::Point2f it_point_world(frame.objects(it_index).world_center().x(),
                        frame.objects(it_index).world_center().y());
                std::vector<cv::Point2f> point_image_obstacle(360), point_world_obstacle(360);
                std::vector<std::pair<float, int> > angle_to_index(360);
                std::vector<int> surround_type_image(360);
                for (int angle_i = 0; angle_i < 360; angle_i++)
                {
                    auto angle_now_rad = (float)(angle_i * CV_PI /180.0f);
                    float dist = 0;
                    cv::Point find_point = it_point + cv::Point((int)(dist*cosf(angle_now_rad)),
                            (int)(dist*sinf(angle_now_rad)));
                    int recognize_step = 0;
                    for (;;)
                    {
                        if (!bound_rect.contains(find_point) || dist > distance_threshold){
                            point_image_obstacle[angle_i] = cv::Point2f(find_point);
                            surround_type_image[angle_i] = rs::NOTHING;
                            break;
                        }
                        auto val_here = frame_mask.at<uchar>(find_point) ;
                        switch (recognize_step){
                            case 0:
                                if (val_here!=220)recognize_step = 1;
                                break;
                            case 1:
                                if (val_here == 220)
                                {
                                    point_image_obstacle[angle_i] = cv::Point2f(find_point);
                                    surround_type_image[angle_i] = rs::CAR;
                                }
                                break;
                        }
                        if(val_here == 0)
                        {
                            point_image_obstacle[angle_i] = cv::Point2f(find_point);
                            surround_type_image[angle_i] = rs::OBSTACLE;
                            break;
                        }
                        else if (recognize_step && val_here == 220)
                            break;
                        dist+=distance_add_step;
                        find_point = it_point + cv::Point((int)(dist*cosf(angle_now_rad)),
                                                                    (int)(dist*sinf(angle_now_rad)));
                    }


                }
                cv::Mat cal_tmp(3,1,CV_32FC1,cv::Scalar(1));
                cv::Mat res_tmp = cal_tmp.clone();
                for (int angle_i = 0; angle_i  < 360 ;angle_i++)
                {
                    cal_tmp.at<float>(0,0) = point_image_obstacle[angle_i].x;
                    cal_tmp.at<float>(1,0) = point_image_obstacle[angle_i].y;
                    res_tmp = base_homograph * cal_tmp;
                    res_tmp/=res_tmp.at<float>(2,0);
                    auto res_world_point_tmp = cv::Point2f(res_tmp.at<float>(0,0),res_tmp.at<float>(1,0));
                    auto relative_point_tmp = res_world_point_tmp - it_point_world;
                    float angle_tmp = atan2(relative_point_tmp.y, relative_point_tmp.x) / (float)CV_PI *180.0f;
                    point_world_obstacle[angle_i] = relative_point_tmp;
                    angle_to_index[angle_i] = std::pair<float, int>(angle_tmp, angle_i);
                }
                std::sort(angle_to_index.begin(), angle_to_index.end());
                /// linear Interpolation !!!
                int low_index = 0;
                for (int angle_i = 0; angle_i < 360; angle_i++)
                {
                    auto iter_surround = iter->add_obj();

                    int angle_to_find = angle_i - 180+1;
                    iter_surround->set_real_angle(angle_to_find);
                    iter_surround->set_relative_angle(angle_to_find-(float)head_angle);
                    while (true)
                    {
                        if (angle_to_index[low_index].first <= angle_to_find &&
                        angle_to_index[low_index+1].first >= angle_to_find)
                        {
                             int obj_index1 = angle_to_index[low_index].second,
                             obj_index2 = angle_to_index[low_index+1].second;
                             if (surround_type_image[obj_index1] == surround_type_image[obj_index2])
                             {
                                 cv::Point2f tmp_point_2f= (point_world_obstacle[obj_index1] *
                                         (angle_to_find - angle_to_index[low_index].first)
                                         + point_world_obstacle[obj_index2] *
                                         (angle_to_index[low_index+1].first - angle_to_find)) /
                                         (angle_to_index[low_index+1].first - angle_to_index[low_index].first);
                                 auto dist = (float)cv::norm(tmp_point_2f);
                                 iter_surround->set_distance(dist);
                                 iter_surround->set_surround_type(surround_type_image[obj_index1]);
                             } else
                             {
                                 float angle_diff_1 =  -angle_to_index[low_index].first + angle_to_find,
                                 angle_diff_2 = angle_to_index[low_index + 1].first - angle_to_find;
                                 if (angle_diff_2 > angle_diff_1)
                                 {
                                     iter_surround->set_distance((float)cv::norm(
                                         point_world_obstacle[angle_to_index[low_index].second]));
                                    iter_surround->set_surround_type(surround_type_image[
                                         angle_to_index[low_index].second]);
                                 }else
                                 {
                                     iter_surround->set_distance((float)cv::norm(
                                             point_world_obstacle[angle_to_index[low_index+1].second]));
                                     iter_surround->set_surround_type(surround_type_image[
                                                                              angle_to_index[low_index+1].second]);
                                 }
                             }
                             break;
                        }
                        else if (angle_to_index[low_index].first > angle_to_find)
                        {
                             if (low_index > 0)
                             {
                                 low_index--;
                             } else{
                                 iter_surround->set_distance((float)cv::norm(
                                         point_world_obstacle[angle_to_index[low_index].second]));
                                 iter_surround->set_surround_type(surround_type_image[
                                         angle_to_index[low_index].second]);
                                 break;
                             }
                        }
                        else if (angle_to_index[low_index+1].first < angle_to_find)
                        {
                            if (low_index+1 < 359)
                            {
                                low_index++;
                            }
                            else
                            {
                                iter_surround->set_distance((float)cv::norm(
                                        point_world_obstacle[angle_to_index[low_index+1].second]));
                                iter_surround->set_surround_type(surround_type_image[
                                                                         angle_to_index[low_index+1].second]);
                                break;
                            }
                        }
                    }

                }

        }
    }
    write_to_binary(&all_trajectory, base_path+"/data/all_trajectories.proto.data");
#else
    read_from_binary(&all_trajectory, base_path + "/data/all_trajectories.proto.data");
    std::cout<<all_trajectory.trajectories_size()<<std::endl;

    //print_one_trajectory(all_trajectory.trajectories(0));
#endif
}

#if 0

void information::print_one_trajectory(const rs::trajectory &trajectory) {
    std::cout<<"trajectory index: "<<trajectory.objects(0).it().object_index()<<std::endl;
    std::cout<<"target position:"<<trajectory.objects(trajectory.objects().size()-1).it().world_center().x()<<", "
    <<trajectory.objects(trajectory.objects().size()-1).it().world_center().y()<<std::endl;
    std::cout<<"trajectory existing time(s): "<<trajectory.objects().size()*1/30.<<std::endl<<std::endl;
    for (auto &item : trajectory.objects())
    {
        std::cout<<"frame index: "<<item.it().frame_index()<<std::endl;
        std::cout<<"current position: "<<item.it().world_center().x()<<", "<<item.it().world_center().y()<<std::endl
        <<"current velocity: "<<item.velocity().x()<<", "<<item.velocity().y()<<std::endl;
        std::cout<<"other objects: "<<std::endl;
        for (auto &item2 : item.obj())
        {
            std::cout<<"distance: "<<item2.distance_to_center()<<", relative angle: "<<item2.angle_to_center()
            <<", object index: "<<item2.index()<<", object position: "<<item2.obj_position().x()
            <<", "<<item2.obj_position().y()<<", velocity: "<<item2.velocity().x()<<", "<<item2.velocity().y()
            <<", object type: "<<item2.surround_type()<<std::endl;
        }
        std::cout<<std::endl;
    }
}



#endif
