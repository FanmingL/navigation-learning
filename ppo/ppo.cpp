/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : ppo.cpp
*   Author      : FanmingL
*   Created date: 2019-01-26 20:21:13
*   Description : 
*
*===============================================================*/


#include "ppo.h"
namespace rs{
namespace al{

    PPO::PPO() :
    config(
            /*const float &_kGamma =*/          0.99f,
            /*const float &_kLambda =*/         0.97f,
            /*const float &_kClip =*/           0.08f,
            /*const float &_kNoise =*/          0.5f,
            /*const float &_kCriticLr =*/       0.001f,
            /*const float &_kActorLr =*/        0.0003f,
            /*const int &_kBatchSize =*/        1000,
            /*const int &_kPointsNum =*/        10000,
            /*const int &_kEpochNum =*/         20,
            /*const int &_kStateLength =*/      544,
            /*const int &_kActionLength =*/     2,
            /*const std::string &_out_path =*/  "./out/",
            /*target kl*/                       0.01f,
            /*real data number =*/              150,
            /*eta = */                          0.005
            ),
            cpu_device(torch::kCPU),
            cuda_device(torch::kCPU),
            e(time(nullptr))
    {
        if (torch::cuda::is_available()){
            std::cout<<"CUDA is Available, training in CUDA!\n";
            cuda_device = torch::Device(torch::kCUDA);
        }
        config.kStateLength = simulator.GetStateDims();
        config.kActionLength = simulator.GetActionDims();
        config.kAgentNum = simulator.GetCarNum();
        config.kLogStd = std::log(config.kNoise);
        trajectories = std::make_shared<TrajectoryBuffer>(cuda_device,
                config.kStateLength,
                config.kActionLength,
                config.kGamma,
                config.kLambda,
                config.kAgentNum,
                config.kPointsNum,
                802);
        sampled_state = torch::zeros({config.kBatchSize, config.kStateLength}, cpu_device).detach();
        sampled_action = torch::zeros({config.kBatchSize, config.kActionLength}, cpu_device).detach();
        sampled_advants = torch::zeros({config.kBatchSize, 1}, cpu_device).detach();
        sampled_return = torch::zeros({config.kBatchSize, 1}, cpu_device).detach();
        real_state = torch::zeros({config.kRealDataNum, config.kStateLength}, cpu_device).detach();
        real_action = torch::zeros({config.kRealDataNum, config.kActionLength}, cpu_device).detach();
        state_now = torch::zeros({config.kAgentNum, config.kStateLength}, cpu_device).detach();
        actor = std::make_shared<ActorModel>(config.kStateLength, cuda_device, "actor_");
        critic = std::make_shared<CriticModel>(config.kStateLength, cuda_device, "critic_");
        actor->to(cuda_device);
        critic->to(cuda_device);
        actor_optimizer = std::make_shared<torch::optim::Adam>(actor->parameters(),
                torch::optim::AdamOptions(config.kActorLr));
        critic_optimizer = std::make_shared<torch::optim::Adam>(critic->parameters(),
                torch::optim::AdamOptions(config.kCriticLr));
        action_now_vector = std::vector<std::vector<float> >(config.kAgentNum,
                std::vector<float>(config.kActionLength, 0));
        index_buffer = std::vector<int>(config.kPointsNum, 0);
        for (int i = 0; i < config.kPointsNum; ++i){
            index_buffer[i] = i;
        }
        std::shuffle(index_buffer.begin(), index_buffer.end(), e);
        normal_distribution = std::normal_distribution<float>(0, config.kNoise);
        int_distribution = std::uniform_int_distribution<int>(0, simulator.GetRealDataLength() - 1);
        ResetEnv();

    }

    std::vector<cs::CarTransition> PPO::RunWithoutSave(float &ret, const bool & need_noise) {
        float *iter = state_now.data<float>();
        for (auto &item : state_now_vector)
            for (auto &item1 : item)
                *(iter ++) = item1;
        auto action_now = actor->forward(state_now.to(cuda_device)).detach().to(cpu_device);
        iter = action_now.data<float>();
        for (auto &item : action_now_vector) {
            for (auto &item1 : item) {
                if(need_noise){
                    item1 = *(iter++) + normal_distribution(e);
                }else {
                    item1 = *(iter++);
                }
            }
        }
        return simulator.Run(action_now_vector, state_now_vector, ret);
    }

    void PPO::ResetEnv() {
        simulator.Reset(state_now_vector);
    }

    bool PPO::RunAndSave(float &ret, const bool &need_noise) {
        auto data = RunWithoutSave(ret, need_noise);
        return trajectories->AddData(data, critic);
    }

    void PPO::SaveModel(int index) {
        actor->to(cpu_device);
        critic->to(cpu_device);

        actor->SaveTo(config.out_path, index);
        critic->SaveTo(config.out_path, index);

        actor->to(cuda_device);
        critic->to(cuda_device);
    }

    void PPO::LoadModel(int index) {
        actor->to(cpu_device);
        critic->to(cpu_device);

        actor->LoadFrom(config.out_path, index);
        critic->LoadFrom(config.out_path, index);

        actor->to(cuda_device);
        critic->to(cuda_device);
        actor_optimizer = std::make_shared<torch::optim::Adam>(actor->parameters(),
                                                               torch::optim::AdamOptions(config.kActorLr));
        critic_optimizer = std::make_shared<torch::optim::Adam>(critic->parameters(),
                                                                torch::optim::AdamOptions(config.kCriticLr));
    }

    float PPO::TrainOneEpoch(float &actor_loss_sum, float &critic_loss_sum) {
        float average_ret = 0.f;
        int average_count = 0;
        actor_loss_sum = critic_loss_sum = 0.f;
        while (true){
            float ret_tmp = 0;
            bool prepared_done = RunAndSave(ret_tmp, true);
            if (ret_tmp != 0){
                average_ret += ret_tmp;
                average_count ++;
            }
            if (prepared_done){
                break;
            }
        }
        auto state_buffer = trajectories->state_buffer.to(cuda_device).detach(),
            action_buffer = trajectories->action_buffer.to(cuda_device).detach(),
            advants_buffer = trajectories->advants_buffer.to(cuda_device).detach(),
            ret_buffer = trajectories->ret_buffer.to(cuda_device).detach(),
            real_state_gt = real_state.to(cuda_device).detach(),
            real_action_gt = real_action.to(cuda_device).detach();

        auto old_policy_action = actor->forward(state_buffer).detach();
        auto old_policy_action_probability = (-(1.f / 2.f / config.kNoise / config.kNoise) *
                                             (action_buffer - old_policy_action).pow(2)).detach();
        float kl_average = 0.f;
        bool optim_actor = true;
        for (int epoch_ = 0; epoch_ < config.kEpochNum; ++epoch_){
            std::shuffle(index_buffer.begin(), index_buffer.end(), e);
            float kl_sum = 0;
            float critic_loss_sum_in = 0, actor_loss_sum_in = 0;
            
            for (int i = 0; i + config.kBatchSize < config.kPointsNum; i += config.kBatchSize)
            {
                auto index_it = torch::arange(i, i + config.kBatchSize, cpu_device).toType(torch::kInt64).detach();
                long *index_iter = (long*)index_it.data_ptr();
                for (int j = i; j <  i + config.kBatchSize ;j++){
                    *(index_iter++) = index_buffer[j];
                }
                index_it = index_it.to(cuda_device);
                sampled_state = state_buffer.index_select(0, index_it).detach();
                sampled_action = action_buffer.index_select(0, index_it).detach();
                sampled_advants = advants_buffer.index_select(0, index_it).detach();
                sampled_return = ret_buffer.index_select(0, index_it).detach();

                auto sampled_old_probability = old_policy_action_probability.index_select(0, index_it).detach();

                if (optim_actor) {
                    
                    auto new_action = actor->forward(sampled_state);
                    auto new_action_probability = -(1.f / 2.f / config.kNoise / config.kNoise) *
                                             (sampled_action - new_action).pow(2);
                    auto kl_diversity = sampled_old_probability - new_action_probability;
                    kl_sum += kl_diversity.sum().item<float>();
                    auto ratio = torch::exp(-kl_diversity);
                    auto actor_loss1 = ratio * sampled_advants;
                    auto actor_loss2 = torch::clamp(ratio, 1- config.kClip, 1 + config.kClip) * sampled_advants;
                    auto actor_loss = -torch::min(actor_loss1, actor_loss2).mean();
                    
                    actor_optimizer->zero_grad();
                    actor_loss.backward(c10::nullopt, true);
                    actor_optimizer->step();
                    //actor_loss_sum_in += actor_mse_loss.item<float>();
                }
                auto predict_value = critic->forward(sampled_state);
                auto critic_loss = torch::mse_loss(predict_value, sampled_return);
                critic_optimizer->zero_grad();
                //critic_loss.backward();
                critic_loss.backward(c10::nullopt, true);
                critic_optimizer->step();
                critic_loss_sum_in += critic_loss.item<float>();
            }
            kl_average = kl_sum / config.kPointsNum;
            critic_loss_sum = critic_loss_sum_in / (config.kPointsNum / config.kBatchSize);
            if (optim_actor){
                GetRealData(real_state, real_action);
                real_state_gt = real_state.to(cuda_device).detach();
                real_action_gt = real_action.to(cuda_device).detach();
                auto predict_action = actor->forward(real_state_gt);
                auto actor_mse_loss =  config.kEta * torch::mse_loss(predict_action, real_action_gt);
                actor_optimizer->zero_grad();
                actor_mse_loss.backward(c10::nullopt, true);
                actor_optimizer->step();
                actor_loss_sum_in = actor_mse_loss.item<float>();
            }
            if (optim_actor){
                //actor_loss_sum = actor_loss_sum_in / (config.kPointsNum / config.kBatchSize);
                actor_loss_sum = actor_loss_sum_in;
            }
            if (optim_actor && kl_average > 1.5f * config.kTargetKl){
                optim_actor = false;
            }

        }
        //actor_loss_sum /= (config.kPointsNum / config.kBatchSize * config.kEpochNum);
        //critic_loss_sum /= (config.kPointsNum / config.kBatchSize * config.kEpochNum);
        ResetEnv();
        return average_ret / average_count;
    }

    void PPO::SetSampleToDevice(const torch::Device &d) {
        sampled_return.to(d);
        sampled_advants.to(d);
        sampled_action.to(d);
        sampled_state.to(d);
    }

    void PPO::GetRealData(torch::Tensor &state, torch::Tensor &action) {
        int count = 0;
        float   *state_iter = state.data<float>(),
                *action_iter = action.data<float>();
        while (true){
            auto data = simulator.GetFromRealData(int_distribution(e));
            if (data.empty())continue;
            for (auto &item : data) {
                for (auto &item1 : item.state){
                    *(state_iter ++)  = item1;
                }
                for (auto &item1 : item.action){
                    *(action_iter ++) = item1;
                }
                count++;
                if (count >= config.kRealDataNum)
                    goto bbb;
            }
        }
        bbb:;
    }

}
}

