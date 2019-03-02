/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : car_simulator.h
*   Author      : FanmingL
*   Created date: 2019-01-12 11:40:24
*   Description : 
*
*===============================================================*/


#ifndef _CAR_SIMULATOR_H
#define _CAR_SIMULATOR_H

#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <random>
#include <thread>
#include <future>

#include "car_information.h"
#include "car_config.h"
#include "data.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"



#ifndef MULTI_CORE
#define MULTI_CORE 0
#endif

namespace rs {
    namespace cs {
        class CarTransition {
        public:
            CarTransition(std::vector<float> &state_,
                          std::vector<float> &action_, const float &reward_, const int &done_) :
                    state(std::move(state_)),
                    action(std::move(action_)), reward(reward_), done(done_) {}

            ~CarTransition() = default;

            friend std::ostream &operator<<(std::ostream &os, const CarTransition &transition) {
                os << "reward: " << transition.reward << " done: " << transition.done << std::endl;
                os << " state: " << std::endl;
                for (auto &item : transition.state) {
                    os << item << ", ";
                }
                os << std::endl << " action: " << std::endl;
                for (auto &item : transition.action) {
                    os << item << ", ";
                }

                return os;
            }

            std::vector<float> state, action;

            float reward;

            int done;
        };

        class CarSimulator {
        public:
            CarSimulator();

            ~CarSimulator() = default;

            void RandomInit(const int &number, std::vector<std::shared_ptr<CarInformation> > &car_list);

            void RunWithoutParam();

            void Reset(std::vector<std::vector<float> > &features);

            void Reset();

            int GetStateDims();

            int GetActionDims();

            int GetCarNum();

            std::vector<CarTransition> Run(std::vector<std::vector<float> > &action,
                                           std::vector<std::vector<float> > &new_state, float &return_value);

            std::vector<CarTransition> GetFromRealData(const int &frame_index);

            void ExecuteAction(std::vector<float> &action, std::shared_ptr<CarInformation> &data);

            int GetRealDataLength();

            CarConfig config;

        private:
            void DrawOnCanvas(cv::Mat &canvas, const std::vector<std::shared_ptr<CarInformation> > &car_list,
                              const float &ratio = 1.0f, const bool &if_add = false);

            void DrawOneCar(cv::Mat &canvas, const std::shared_ptr<CarInformation> &data,
                            const float &ratio = 1.0f, const bool &if_add = false);

            bool CheckOverLap(const cv::Mat &canvas, const CarInformation &data,
                              const float &ratio = 1.0f, const bool &only_obstacle = false);

            bool CheckOverLap(const cv::Mat &canvas, const std::shared_ptr<CarInformation> &data,
                              const float &ratio = 1.0f, const bool &only_obstacle = false);

            bool CheckOverLap(const cv::Mat &canvas, const float &x_, const float &y_, const float &angle,
                              const int &index, const float &ratio = 1.0f, const bool &only_obstacle = false);

            bool ShowImage(const std::vector<std::shared_ptr<CarInformation> > &car_list);

            void ExecuteAllActions(std::vector<std::vector<float> > &action,
                                   std::vector<std::shared_ptr<CarInformation> > &data);


            void CheckAllActions(const cv::Mat &canvas, std::vector<std::shared_ptr<CarInformation> > &data);

            void CheckAction(const cv::Mat &canvas, std::shared_ptr<CarInformation> &data);

            float RunSummary(std::vector<std::shared_ptr<CarInformation> > &car_data);

            bool RunSummaryInit(std::vector<std::shared_ptr<CarInformation> > &car_data);

            void ExtractAllFeatures(const std::vector<std::shared_ptr<CarInformation> > &data,
                                    cv::Mat &canvas, std::vector<std::vector<float> > &features);

            void ExtractFeature(cv::Mat &canvas, const std::shared_ptr<CarInformation> &data);

            void AppendOneHot(std::vector<float> &feature, const int &t);

            void ZeroPadding(cv::Mat &background);

            float LimitAngleVelocity(const float &w, const float &v);

            std::default_random_engine e;

            std::uniform_real_distribution<float> float_uniform;

            inline cv::Point2f
            GetConvertPoint(const float &x, const float &y, const float &cos_angle, const float &sin_angle,
                            const int &i, const int &j);


            cv::Mat background, background_int, canvas, image, background_without_padding, background_without_padding_int;

            std::vector<std::shared_ptr<CarInformation> > car_list;

            std::unordered_set<int> dead_index;

            cv::Rect max_rect;


            std::normal_distribution<float> float_normal, init_angle_normal;

            int max_index;

            cv::VideoWriter video_writer;

            std::unordered_map<int, float> x_velocity_buff, y_velocity_buff;

            float angle_normalization_coeff, velocity_normalization_coeff, angle_action_normalization_coeff,
                    velocity_action_normalization_coeff, distance_normalization_coeff;
            /* inverse */
            float angle_normalization_coeff_d, velocity_normalization_coeff_d, angle_action_normalization_coeff_d,
                    velocity_action_normalization_coeff_d, distance_normalization_coeff_d, distance_normalization_mean,
                    target_distance_coeff_d, target_distance_mean, reward_normalization_coeff_d;

            AllFrameCarInformation all_data;

        };
    }
}

#endif //CAR_SIMULATOR_H
