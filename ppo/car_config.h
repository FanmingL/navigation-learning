//
// Created by Fanming Luo on 2019/1/12.
//

#ifndef CAR_SIMULATOR_CAR_CONFIG_H
#define CAR_SIMULATOR_CAR_CONFIG_H

#include <string>
#include <iostream>

namespace rs {
    namespace cs {
        class CarConfig {
        public:
            CarConfig();

            friend std::ostream &operator<<(std::ostream &os, const CarConfig &config);

            ~CarConfig() = default;

            int car_number, lidar_max_count, type_number, thread_number, lidar_angle_number, zero_padding;

            bool if_show_image, if_draw_lidar, if_draw_target, if_write_video;

            std::string background_path, out_video_path, real_data_path;

            float init_cov, car_width, car_height, padding, lidar_max_distance,
                    lidar_distance_interval, min_velocity, max_trajectory_length, pixels_per_meter,
                    reward_too_long, reward_collision, reward_get_target, reward_run_ratio, lidar_angle_interval,
                    max_velocity, max_angle_velocity, max_velocity_change, init_velocity_std,
                    min_rotate_radius, one_step_reward;
            bool if_read_real_data, if_test_mode;

        };
    }
}

#endif //CAR_SIMULATOR_CAR_CONFIG_H
