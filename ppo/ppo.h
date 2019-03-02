/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : ppo.h
*   Author      : FanmingL
*   Created date: 2019-01-26 20:21:15
*   Description : 
*
*===============================================================*/


#ifndef _PPO_H
#define _PPO_H


#include "car_simulator.h"
#include "ppo_config.h"
#include "trajectory_buffer.h"
#include "model.h"
namespace rs{
    namespace al{
        class PPO{
        public:
            PPO();

            ~PPO() = default;

            void ResetEnv();

            std::vector<cs::CarTransition> RunWithoutSave(float &ret, const bool &need_noise);

            bool RunAndSave(float &ret, const  bool &need_noise);

            void SaveModel(int index);

            void LoadModel(int index);

            float TrainOneEpoch(float &actor_loss, float &critic_loss);

            void SetSampleToDevice(const torch::Device& d);

            void GetRealData(torch::Tensor &state, torch::Tensor &action);
        private:

            cs::CarSimulator simulator;
            PPOConfig config;

            std::shared_ptr<ActorModel> actor;
            std::shared_ptr<CriticModel> critic;
            std::shared_ptr<torch::optim::Adam> actor_optimizer, critic_optimizer;
            torch::Device cpu_device, cuda_device;
            std::shared_ptr<TrajectoryBuffer> trajectories;

            torch::Tensor sampled_state, sampled_action, sampled_advants, sampled_return, state_now,
            real_state, real_action;

            std::vector<std::vector<float> > action_now_vector, state_now_vector;
            std::vector<int> index_buffer;
            std::default_random_engine e;
            std::normal_distribution<float> normal_distribution;
            std::uniform_int_distribution<int> int_distribution;

        };
    }
}
#endif //PPO_H
