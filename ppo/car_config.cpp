//
// Created by Fanming Luo on 2019/1/12.
//


#include "car_config.h"

namespace rs {
    namespace cs {

        CarConfig::CarConfig() {
            car_number = 10;
            if_test_mode = true;

            if_show_image = if_test_mode;
            if_read_real_data = !if_test_mode;

            lidar_max_count = (int) (20 / 0.5f);
            type_number = 3;
            thread_number = 3;
            lidar_angle_number = 45;
            zero_padding = 3;
            if_draw_target = false;
            if_write_video = true;
            if_draw_lidar = false;

            /* string value config */
            out_video_path = "out_video.mp4";
            background_path = "./config/background_env.png";
            real_data_path = "./config/real-data.txt";

            /* float value config */
            pixels_per_meter = 4;
            init_cov = 0.8f;
            car_width = 2 * pixels_per_meter;
            car_height = 3 * pixels_per_meter;
            padding = 8 * pixels_per_meter;
            lidar_max_distance = 25 * pixels_per_meter;
            lidar_distance_interval = 0.5f * pixels_per_meter;
            max_velocity = 4;
            //min_velocity = 0.12;
            min_velocity = -max_velocity;
            max_trajectory_length = 800;
            reward_collision = -1000;
            reward_get_target = 1000;
            reward_run_ratio = 3.0f;
            reward_too_long = 0;
            lidar_angle_interval = 3.1415926535f * 2.f / lidar_angle_number;
            max_angle_velocity = 4;
            max_velocity_change = 0.8;
            init_velocity_std = 0.4f;
            min_rotate_radius = 5.f * pixels_per_meter;
            one_step_reward = -1.f / max_trajectory_length;
        }

        std::ostream &operator<<(std::ostream &os, const CarConfig &config) {
            os << "car_number: " << config.car_number << " lidar_max_count: " << config.lidar_max_count
               << " type_number: " << config.type_number << " thread_number: " << config.thread_number
               << " lidar_angle_number: " << config.lidar_angle_number << " zero_padding: " << config.zero_padding
               << " if_show_image: " << config.if_show_image << " if_draw_lidar: " << config.if_draw_lidar
               << " if_draw_target: " << config.if_draw_target << " if_write_video: " << config.if_write_video
               << " background_path: " << config.background_path << " out_video_path: " << config.out_video_path
               << " init_cov: " << config.init_cov << " car_width: " << config.car_width << " car_height: "
               << config.car_height << " padding: " << config.padding << " lidar_max_distance: "
               << config.lidar_max_distance << " lidar_distance_interval: " << config.lidar_distance_interval
               << " min_velocity: " << config.min_velocity << " max_trajectory_length: " << config.max_trajectory_length
               << " pixels_per_meter: " << config.pixels_per_meter << " reward_too_long: " << config.reward_too_long
               << " reward_collision: " << config.reward_collision << " reward_get_target: " << config.reward_get_target
               << " reward_run_ratio: " << config.reward_run_ratio << " lidar_angle_interval: "
               << config.lidar_angle_interval;
            return os;
        }


    }
}
