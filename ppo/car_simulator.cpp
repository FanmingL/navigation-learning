/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : car_simulator.cpp
*   Author      : FanmingL
*   Created date: 2019-01-12 11:40:18
*   Description : 
*
*===============================================================*/


#include "car_simulator.h"
#ifndef GO_STRAIGHT
#define GO_STRAIGHT 1
#endif
#ifndef ALL_SAME
#define ALL_SAME 1
#endif
namespace rs {
    namespace cs {
        CarSimulator::CarSimulator() :e((unsigned int) time(nullptr)),
                                                              float_uniform(0, 1), max_index(40000),
                                                              init_angle_normal(0, 1)
                                                              {
            for (int i = 0; i < config.car_number; i++){
                dead_index.insert(i);
            }
            car_list = std::vector<std::shared_ptr<CarInformation> >(config.car_number);
            float_normal = std::normal_distribution<float>(0, config.init_cov);
            init_angle_normal = std::normal_distribution<float>(0, config.init_velocity_std);
            background = cv::imread(config.background_path, cv::IMREAD_GRAYSCALE);
            background.convertTo(background_without_padding, CV_32SC1);
            if (config.if_read_real_data) {
                std::ifstream iff_(config.real_data_path);
                all_data.init(iff_, config.pixels_per_meter, max_index);
            }
            ZeroPadding(background);
            background.convertTo(background_int, CV_32SC1);
            background.copyTo(image);
            max_rect = cv::Rect(0, 0, background.cols, background.rows);
            RandomInit(config.car_number, car_list);

            //float angle_normalization_coeff, velocity_normalization_coeff, angle_action_normalization_coeff,
            //velocity_action_normalization_coeff, distance_normalization_coeff;
            angle_normalization_coeff = (float) CV_PI;
            angle_action_normalization_coeff = config.max_angle_velocity;
            velocity_action_normalization_coeff = config.max_velocity_change;
            velocity_normalization_coeff = config.max_velocity;
            distance_normalization_coeff = config.lidar_max_distance;

            angle_normalization_coeff_d = 1.f / angle_normalization_coeff;
            angle_action_normalization_coeff_d = 1.f / angle_action_normalization_coeff;
            velocity_action_normalization_coeff_d = 1.f / velocity_action_normalization_coeff;
            velocity_normalization_coeff_d = 1.f / velocity_normalization_coeff;

            distance_normalization_coeff_d = 2.f / distance_normalization_coeff;
            distance_normalization_mean = config.lidar_max_distance / 2.f;

            target_distance_mean = MyNorm((float) (max_rect.width), (float) (max_rect.height)) / 2.f;
            target_distance_coeff_d = 1.f / MyNorm((float) (max_rect.width), (float) (max_rect.height));
            reward_normalization_coeff_d = 1.f / 1000.f;
            if (config.if_write_video){
                std::remove((char*)(config.out_video_path.c_str()));
                video_writer.open(config.out_video_path,
                        CV_FOURCC('D', 'I', 'V', 'X'), 30,
                        cv::Size(background.cols, background.rows), false);
            }

        }

        void CarSimulator::Reset(std::vector<std::vector<float> > &features) {
            //car_list.clear();
            for (int i = 0; i < config.car_number; ++i){
                dead_index.insert(i);
            }
            RandomInit(config.car_number, car_list);
            canvas.convertTo(image, CV_8U);
            ExtractAllFeatures(car_list, canvas, features);
            ShowImage(car_list);
        }

        void CarSimulator::RandomInit(const int &number, std::vector<std::shared_ptr<CarInformation> > &car_list) {
            if (dead_index.empty()){
                for (auto &item : car_list) {
                    x_velocity_buff[item->index] = item->vx;
                    y_velocity_buff[item->index] = item->vy;
                }
                return;
            }
            background_int.copyTo(canvas);
            DrawOnCanvas(canvas, car_list, 2);
            while (!dead_index.empty()) {
                int it_count = *(dead_index.begin());
                std::shared_ptr<CarInformation> car = std::make_shared<CarInformation>(
                        float_uniform(e) * (max_rect.width - config.padding) + config.padding / 2,
                        float_uniform(e) * (max_rect.height - config.padding) + config.padding / 2,
                        float_normal(e),
                        float_normal(e),
                        float_uniform(e) * (max_rect.width - config.padding) + config.padding / 2,
                        float_uniform(e) * (max_rect.height - config.padding) + config.padding / 2,
                        max_index - 1 - it_count
                );
                if ((car->x <= 100 && car->target_x >= 107) || (car->x >= 107 && car->target_x <= 100))
                    continue;
                if (MyNorm(car->x - car->target_x, car->y - car->target_y) <=
                    2.f * MyNorm(config.car_width, config.car_height)) {
                    continue;
                }
#if GO_STRAIGHT
                float t_theta = atan2(car->target_y-car->y, car->target_x-car->x);
                float delta1 = std::abs(t_theta - (float)CV_PI/2.f);
#if ALL_SAME
                float delta2 = std::abs(t_theta - (float)CV_PI/2.f);
#else
                float delta2 = std::abs(t_theta + (float)CV_PI/2.f);
#endif
                if (delta1> 8 / 180.f *3.1415 && delta2 > 8/180.f*3.1415)
                	continue;
#endif
                if (config.if_test_mode && it_count == 0 ){
                    car->x = (35 + float_normal(e)) * config.pixels_per_meter;
                    car->y = (50 + float_normal(e)) * config.pixels_per_meter;
                    car->target_x = 29 * config.pixels_per_meter;
                    car->target_y = 4 * config.pixels_per_meter;
                }
                if (config.if_test_mode && it_count != 0 ){
                    car->x = 29 * config.pixels_per_meter;
                    car->target_x = 29 * config.pixels_per_meter;
                    car->target_y = 4 * config.pixels_per_meter;
                }

                if (config.if_test_mode && CheckOverLap(canvas, car, 1.0f, false) &&
                    CheckOverLap(canvas, car->target_x, car->target_y, 0, 0, 1, true)) {
                    DrawOneCar(canvas, car, 1);
                    float v_abs = MyNorm(car->vx, car->vy);
                    v_abs = MyLimit(v_abs, config.min_velocity, config.max_velocity);
                    float angle_target = atan2(car->target_y - car->y, car->target_x - car->x);
                    float angle = angle_target + init_angle_normal(e);
                    if (config.if_test_mode){
                        angle = angle_target;
                    }
                    car->vx = v_abs * cosf(angle);
                    car->vy = v_abs * sinf(angle);
                    car->angle = angle;
                    car_list[it_count] = car;
                    dead_index.erase(it_count);
                }
                else if (CheckOverLap(canvas, car, 2.0f, false) &&
                    CheckOverLap(canvas, car->target_x, car->target_y, 0, 0, 2, true)) {
                    DrawOneCar(canvas, car, 2);
                    float v_abs = MyNorm(car->vx, car->vy);
                    v_abs = MyLimit(v_abs, config.min_velocity, config.max_velocity);
                    float angle_target = atan2(car->target_y - car->y, car->target_x - car->x);
                    float angle = angle_target + init_angle_normal(e);
                    if (config.if_test_mode){
                        angle = angle_target;
                    }
                    car->vx = v_abs * cosf(angle);
                    car->vy = v_abs * sinf(angle);
                    car->angle = angle;
                    car_list[it_count] = car;
                    dead_index.erase(it_count);
                }
            }

            for (auto &item : car_list) {
                x_velocity_buff[item->index] = item->vx;
                y_velocity_buff[item->index] = item->vy;
            }
            background_int.copyTo(canvas);
            DrawOnCanvas(canvas, car_list, 1);
        }

        /// TODO: parallel
        void CarSimulator::DrawOnCanvas(cv::Mat &canvas, const std::vector<std::shared_ptr<CarInformation> > &car_list,
                                        const float &ratio, const bool &if_add) {
            for (int i = 0; i < car_list.size(); ++i) {
                if (dead_index.count(i))
                    continue;
                auto &item = car_list[i];
                DrawOneCar(canvas, item, ratio, if_add);
            }
        }

        void CarSimulator::DrawOneCar(cv::Mat &canvas, const std::shared_ptr<CarInformation> &data,
                                      const float &ratio, const bool &if_add) {
            int car_x_lim = (int) round(config.car_width / 2 * ratio);
            int car_y_lim = (int) round(config.car_height / 2 * ratio);
            float cos_angle = cosf(data->angle), sin_angle = sinf(data->angle);
            for (int i = -car_x_lim; i <= car_x_lim; ++i) {
                for (int j = -car_y_lim; j <= car_y_lim; ++j) {
                    auto p = GetConvertPoint(data->x, data->y, cos_angle, sin_angle, i, j);
                    auto &d = canvas.at<int>(p);
                    if (max_rect.contains(p)) {
                        if (if_add) {
                            if (d != data->index) {
                                if (d == max_index) {
                                    d = data->index;
                                } else {
                                    d += data->index;
                                }
                            }
                        } else {
                            d = data->index;
                        }
                    }
                }
            }
        }

        cv::Point2f
        CarSimulator::GetConvertPoint(const float &x, const float &y, const float &cos_angle, const float &sin_angle,
                                      const int &i, const int &j) {
            float x_ = x + j * cos_angle + i * sin_angle, y_ = y + j * sin_angle - i * cos_angle;
            return cv::Point2f(x_, y_);
        }

        bool CarSimulator::CheckOverLap(const cv::Mat &canvas, const std::shared_ptr<CarInformation> &data,
                                        const float &ratio, const bool &only_obstacle) {
            return CheckOverLap(canvas, data->x, data->y, data->angle, data->index, ratio, only_obstacle);
        }

        bool CarSimulator::CheckOverLap(const cv::Mat &canvas, const float &x_, const float &y_, const float &angle_,
                                        const int &index_, const float &ratio, const bool &only_obstacle) {
            int car_x_lim = (int) round(config.car_width / 2 * ratio);
            int car_y_lim = (int) round(config.car_height / 2 * ratio);
            float cos_angle = cosf(angle_), sin_angle = sinf(angle_);
            for (int i = -car_x_lim; i <= car_x_lim; ++i) {
                for (int j = -car_y_lim; j <= car_y_lim; ++j) {
                    auto p = GetConvertPoint(x_, y_, cos_angle, sin_angle, i, j);
                    if (!max_rect.contains(p))continue;
                    int d = canvas.at<int>(p);
                    if (only_obstacle) {
                        if (d == 128)
                            return false;
                    } else {
                        if ((d != 0) && (d != max_index) && (d != index_)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        bool CarSimulator::CheckOverLap(const cv::Mat &canvas, const CarInformation &data, const float &ratio,
                                        const bool &only_obstacle) {
            return CheckOverLap(canvas, data.x, data.y, data.angle, data.index, ratio, only_obstacle);
        }

        bool CarSimulator::ShowImage(const std::vector<std::shared_ptr<CarInformation> > &car_list) {

            if (config.if_show_image) {
                if (config.if_draw_target) {
                    for (auto &item : car_list) {
                        cv::line(image, cv::Point2f(item->x, item->y), cv::Point2f(item->target_x, item->target_y),
                                 cv::Scalar(255));
                    }
                }
                /*
                for (int i = 0; i < car_list.size(); i++){
                    cv::putText(image, std::to_string(i), cv::Point2f(car_list[i]->x, car_list[i]->y), cv::FONT_ITALIC, 1.0,
                            cv::Scalar(255));
                }*/
                if (config.if_write_video){
                    video_writer << image;
                }

                cv::imshow("show image", image);
                return (cv::waitKey(1) == 'q');
            }
            return true;
        }

        /*
         * core code
         * step
         */
        std::vector<CarTransition>
        CarSimulator::Run(std::vector<std::vector<float> > &action, std::vector<std::vector<float> > &new_state,
                          float &return_value) {
            /* execute action */
            ExecuteAllActions(action, car_list);
            /* draw canvas */
            background_int.copyTo(canvas);
            DrawOnCanvas(canvas, car_list, 1, true);
            /* check collision and trajectory length */
            CheckAllActions(canvas, car_list);
            /* new */
            /* construct result and delete some items */
            std::vector<CarTransition> res;

            return_value = RunSummary(car_list);
            for (int i = 0; i < car_list.size(); ++i){
                int done = 0;
                if (dead_index.count(i))
                    done = 1;
                if (car_list[i]->count >= config.max_trajectory_length)
                    done = -1;
                res.emplace_back(car_list[i]->state, car_list[i]->action, car_list[i]->reward, done);
            }
            RandomInit(config.car_number, car_list);
            /* add items so that the total number of cars on canvas are conserved */
            canvas.convertTo(image, CV_8U);
            /* extracted features */

            ExtractAllFeatures(car_list, canvas, new_state);

            ShowImage(car_list);

            return std::move(res);
        }

        /// TODO: parallel
        void CarSimulator::ExecuteAllActions(std::vector<std::vector<float> > &action,
                                             std::vector<std::shared_ptr<CarInformation> > &data) {
            int count = 0;
            for (auto &item : data) {
                ExecuteAction(action[count++], item);
            }
        }

        void CarSimulator::ExecuteAction(std::vector<float> &action, std::shared_ptr<CarInformation> &data) {
            auto abs_velocity = MyNorm(data->vx, data->vy);
            action[0] = MyLimit(action[0], -1.f, 1.f);
            abs_velocity += action[0] * velocity_action_normalization_coeff;
            /// limit car's min velocity
            if (abs_velocity < 0){
                data->angle += 3.141592654f;
                abs_velocity = -abs_velocity;
            }
            abs_velocity = MyLimit(abs_velocity, config.min_velocity, config.max_velocity);
            action[1] = MyLimit(action[1], -1.f, 1.f);
            float w = LimitAngleVelocity(action[1] * angle_action_normalization_coeff * 0.01745329f, abs_velocity);
            //std::cout<<w<<std::endl;
            data->angle += w;
            data->init_distance_to_target = MyNorm(data->x - data->target_x, data->y - data->target_y);
            data->vx = abs_velocity * cosf(data->angle);
            data->vy = abs_velocity * sinf(data->angle);
            data->x += data->vx;
            data->y += data->vy;
            data->action = action;
        }

        /// TODO: parallel
        void CarSimulator::CheckAllActions(const cv::Mat &canvas, std::vector<std::shared_ptr<CarInformation> > &data) {
            for (auto &item : data) {
                CheckAction(canvas, item);
            }
        }

        void CarSimulator::CheckAction(const cv::Mat &canvas, std::shared_ptr<CarInformation> &data) {
            data->reward = ((-MyNorm(data->x - data->target_x, data->y - data->target_y) +
                             data->init_distance_to_target) * config.reward_run_ratio
                            * reward_normalization_coeff_d + config.one_step_reward) * 0.33f;
            if (!CheckOverLap(canvas, data, 1, false)) {
                data->mask = 0;
                data->reward = config.reward_collision * reward_normalization_coeff_d * 0.33f;
                data->return_value += data->reward;
                return;
            }
            cv::Point2f p(data->target_x, data->target_y);
            if (canvas.at<int>(p) == data->index) {
                data->mask = 0;
                data->reward = config.reward_get_target * reward_normalization_coeff_d * 0.33f;
                data->return_value += data->reward;
                return;
            }
            if (data->count >= config.max_trajectory_length) {
                if (config.reward_too_long == 0) {
                    data->mask = 2;
                    data->return_value += data->reward;
                } else {
                    data->mask = 0;
                    data->reward = config.reward_too_long * reward_normalization_coeff_d * 0.33f;
                    data->return_value += data->reward;
                }
                return;
            }
            data->mask = 1;
            data->return_value += data->reward;
            data->count++;
            //data->memory.emplace_back(data->state, data->action, data->reward, data->mask);
        }

        float CarSimulator::RunSummary(std::vector<std::shared_ptr<CarInformation> > &car_data) {
            std::vector<float> return_value;
            for (int i = 0; i < car_data.size(); ++i) {
                auto &item = car_data[i];
                if (item->mask == 0 || item->count >= config.max_trajectory_length) {
                    return_value.push_back(item->return_value);
                    dead_index.insert(i);
                }
            }
            if (!return_value.empty()) {
                float res = 0;
                for (auto &item : return_value)
                    res += item;
                return (res / return_value.size());
            }
            return 0;
        }

        bool CarSimulator::RunSummaryInit(std::vector<std::shared_ptr<CarInformation> > &car_data) {
            for (auto &item : car_data) {
                if (item->mask == 0 || item->count >= config.max_trajectory_length) {
                    return true;
                }
            }
            return false;
        }

        /// TODO: parallel
        void CarSimulator::ExtractAllFeatures(const std::vector<std::shared_ptr<CarInformation> > &data, cv::Mat &canvas,
                                              std::vector<std::vector<float> > &features) {
            features.clear();
            for (auto &item : data) {
                ExtractFeature(canvas, item);
            }
            for (auto &item : data) {
                features.push_back(item->state);
            }
        }

        void CarSimulator::ExtractFeature(cv::Mat &canvas, const std::shared_ptr<CarInformation> &data) {
            float target_angle = atan2(data->target_y - data->y, data->target_x - data->x) - data->angle,
                    target_distance = MyNorm(data->target_x - data->x, data->target_y - data->y),
                    abs_velocity = MyNorm(data->vx, data->vy);
            float sin_angle = sinf(data->angle), cos_angle = cosf(data->angle);
            if (target_angle > CV_PI) target_angle -= 2 * (float) CV_PI;
            if (target_angle < -CV_PI) target_angle += 2 * (float) CV_PI;
            std::vector<float> feature;
            //feature.push_back(target_angle * angle_normalization_coeff_d) ;
            feature.push_back((target_distance) * target_distance_coeff_d);
            feature.push_back(abs_velocity * velocity_normalization_coeff_d);
            feature.push_back(cosf(target_angle)); // y
            feature.push_back(sinf(target_angle));// x
            //feature.push_back(((float)data->count) / config.max_trajectory_length);// x
            for (int i = 0; i < config.lidar_angle_number; i++) {
                float angle_now = i * config.lidar_angle_interval + data->angle;
                float cos_angle_now = cosf(angle_now), sin_angle_now = sinf(angle_now);
                int type = 0;//indicate nothing
                float vx = 0, vy = 0, distance = config.lidar_max_distance;  //nothing
                for (int j = 0; j < config.lidar_max_count; j++) {
                    cv::Point2f p(data->x + j * config.lidar_distance_interval * cos_angle_now,
                                  data->y + j * config.lidar_distance_interval * sin_angle_now);
                    if (!max_rect.contains(p))
                        break;
                    int &d = canvas.at<int>(p);
                    if (config.if_draw_lidar && data->index % 3 == 0) {
                        image.at<uchar>(p) = 255;
                    }
                    if (d != 0 && d != max_index && d != data->index) {
                        if (d == 128) {
                            /* obstacle */
                            type = 1;
                            vx = 0;
                            vy = -abs_velocity;
                        } else {
                            type = 2;
                            float vxtt = x_velocity_buff[d], vytt = y_velocity_buff[d];
                            vx = vxtt * sin_angle - vytt * cos_angle;
                            vy = vxtt * cos_angle + vytt * sin_angle - abs_velocity;
                        }
                        distance = j * config.lidar_distance_interval;
                        break;
                    }
                }
                AppendOneHot(feature, type);
                feature.push_back((distance) * distance_normalization_coeff_d);
                feature.push_back(vx * velocity_normalization_coeff_d);
                feature.push_back(vy * velocity_normalization_coeff_d);
            }
            data->state = std::move(feature);
        }

        void CarSimulator::AppendOneHot(std::vector<float> &feature, const int &t) {
            for (int i = 0; i < config.type_number; ++i) {
                if (t == i) {
                    feature.push_back(1);
                } else {
                    feature.push_back(0);
                }
            }
        }

        void CarSimulator::ZeroPadding(cv::Mat &background) {
            cv::Rect rect(config.zero_padding - 1, config.zero_padding - 1,
                          background.cols - 2 * config.zero_padding, background.rows - 2 * config.zero_padding);
            if (config.if_test_mode){
                rect = cv::Rect(21 * config.pixels_per_meter , config.zero_padding - 1,
                        18 * config.pixels_per_meter, background.rows - 2 * config.zero_padding);
            }
            for (int i = 0; i < background.rows; ++i) {
                auto *iter = background.ptr<uchar>(i);
                for (int j = 0; j < background.cols; ++j) {
                    cv::Point p(j, i);
                    if (!rect.contains(p)) {
                        *(iter + j) = 128;
                    }
                }
            }
            if (config.if_test_mode){
                rect = cv::Rect(33 * config.pixels_per_meter, 0,
                                8 * config.pixels_per_meter, 18 * config.pixels_per_meter);
                for (int i = 0; i < background.rows; ++i) {
                auto *iter = background.ptr<uchar>(i);
                for (int j = 0; j < background.cols; ++j) {
                    cv::Point p(j, i);
                    if (rect.contains(p) && (j - 0.8 * i) > 22* config.pixels_per_meter) {
                        *(iter + j) = 128;
                    }
                }
                }
            }
        }

        void CarSimulator::RunWithoutParam() {
            std::vector<std::vector<float> > actor(config.car_number, std::vector<float>(2)), state;
            std::uniform_real_distribution<float> uniform1(-1.f, 1.f);
            std::uniform_real_distribution<float> uniform2(-0.6f, 0.6f);
            for (auto &item : actor) {
                item[0] = uniform1(e);
                item[1] = uniform2(e);
            }
            float return_value;
            auto res = Run(actor, state, return_value);
            //std::cout<<GetCarNum()<<", "<<res.size()<<std::endl;
            /*auto & item = res[2];
            std::cout<<item.state[2]<<", "<<item.state[3]<<", "<<item.done
            <<", "<<item.action[0]<<", "<<item.action[1]
            <<std::endl;*/
        }

        void CarSimulator::Reset() {
            std::vector<std::vector<float> > state;
            Reset(state);
        }

        int CarSimulator::GetStateDims() {
            return 6 * config.lidar_angle_number + 4;
        }

        int CarSimulator::GetActionDims() {
            return 2;
        }

        int CarSimulator::GetCarNum() {
            return config.car_number;
        }

        float CarSimulator::LimitAngleVelocity(const float &w, const float &v) {
            if (w > v / config.min_rotate_radius) {
                return v / config.min_rotate_radius;
            }
            if (w < -v / config.min_rotate_radius) {
                return -v / config.min_rotate_radius;
            }
            return w;
        }

        std::vector<CarTransition> CarSimulator::GetFromRealData(const int &frame_index){
            if (!config.if_read_real_data) return std::vector<CarTransition>();
            if (frame_index < 0) return std::vector<CarTransition>();
            if (frame_index >= all_data.data.size()) return std::vector<CarTransition>();
            if (all_data.data[frame_index].data.empty())return std::vector<CarTransition>();
            auto &data_tmp = all_data.data[frame_index].data;
            std::vector<std::shared_ptr<CarInformation> > real_car;
            real_car.reserve(data_tmp.size());
            for (auto &item : data_tmp){
                real_car.emplace_back(std::make_shared<CarInformation>(
                        item.x,
                        item.y,
                        item.vx,
                        item.vy,
                        item.target_x,
                        item.target_y,
                        item.index
                ));
                real_car.back()->action.push_back((MyNorm(item.next_vx, item.next_vy) - MyNorm(item.vx, item.vy))
                * velocity_action_normalization_coeff_d);
                float theta =atan2(item.next_vy, item.next_vx) / (float)CV_PI * 180.f - item.angle;
                if (theta > 180)theta -= 360;
                if (theta < -180)theta += 360;
                theta *= angle_action_normalization_coeff_d;
                real_car.back()->action.push_back(theta);
                x_velocity_buff[item.index] = item.vx;
                y_velocity_buff[item.index] = item.vy;
            }

            background_without_padding.copyTo(background_without_padding_int);
            DrawOnCanvas(background_without_padding_int, real_car, 1, false);
            std::vector<CarTransition> res;
            for (auto &item : real_car){
                ExtractFeature(background_without_padding_int, item);
                std::vector<float> action_(2);
                res.emplace_back(item->state, item->action, 0, 0);
            }
            for (auto &item : data_tmp){
                x_velocity_buff.erase(item.index);
                y_velocity_buff.erase(item.index);
            }
            /*
            background_without_padding_int.convertTo(image, CV_8U);
            ShowImage(real_car);
             */
            return std::move(res);
        }

        int CarSimulator::GetRealDataLength(){
            return (int)all_data.data.size();
        }

    }
}
