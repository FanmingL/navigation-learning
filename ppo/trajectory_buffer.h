/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : trajectory_buffer.h
*   Author      : FanmingL
*   Created date: 2019-01-26 20:24:21
*   Description : 
*
*===============================================================*/


#ifndef _TRAJECTORY_BUFFER_H
#define _TRAJECTORY_BUFFER_H

#include <torch/torch.h>
#include <model.h>
#include "car_simulator.h"

namespace rs{
    namespace al{
        class TrajectoryPoints{
        public:
            TrajectoryPoints(torch::Device &d, int state_length_, int action_length_,
                    int max_trajectory_length, float kGamma_, float kLambda_) : count(0), cuda_device(d),
                    cpu_device(torch::kCPU),kGamma(kGamma_), kLambda(kLambda_),
                    state_length(state_length_), action_length(action_length_)
                    {
                state = torch::zeros({max_trajectory_length, state_length}, cpu_device).detach();
                action = torch::zeros({max_trajectory_length, action_length}, cpu_device).detach();
                reward = torch::zeros({max_trajectory_length, 1}, cpu_device).detach();
                advantage = torch::zeros({max_trajectory_length, 1}, cpu_device).detach();
                ret = torch::zeros({max_trajectory_length, 1}, cpu_device).detach();
                Clear();

            }

            ~TrajectoryPoints() = default;

            void Clear(){
                count = 0;
                SetPtr(0);
            }

            void SetPtr(int index = 0){
                state_ptr = state[index].data<float>();
                action_ptr = action[index].data<float>();
                reward_ptr = reward[index].data<float>();
            }

            int AddData(const std::vector<float> &state_vec, const std::vector<float> &action_vec,
                    const float &reward_v, const float &done_v, std::shared_ptr<CriticModel> &model,
                    torch::Tensor &state_buffer, torch::Tensor &action_buffer,
                    torch::Tensor &advants_buffer, torch::Tensor &ret_buffer, int max_len, int start_index){
                if (done_v == -1){
                    Clear();
                    return -1;
                }
                for (auto &item : state_vec)
                    *(state_ptr++) = item;
                for (auto &item : action_vec)
                    *(action_ptr++) = item;
                *(reward_ptr++) = reward_v;
                count++;
                if (done_v == 1){
                    reward_ptr --;
                    auto value = model->forward((state.index_select(0,
                            torch::arange(0, count).toType(torch::kInt64))).to(cuda_device)).to(cpu_device);
                    float *ret_ptr = ret[count - 1].data<float>(),
                        *advantage_ptr = advantage[count - 1].data<float>(),
                        *value_ptr = value[count - 1].data<float>();
                    float running_ret = 0, previous_value = 0, running_advants = 0;
                    for (int i = count - 1; i >= 0; i--){
                        running_ret = *(reward_ptr) + kGamma * running_ret ;
                        running_advants = *(reward_ptr) + kGamma * previous_value - *(value_ptr)
                                + kGamma * kLambda *running_advants;

                        *(ret_ptr) = running_ret;
                        previous_value = *(value_ptr);
                        *(advantage_ptr) = running_advants;
                        ret_ptr--;
                        advantage_ptr--;
                        value_ptr--;
                        reward_ptr --;
                    }
                    int cp_length = std::min(max_len - start_index, count);
                    float *state_dst = state_buffer[start_index].data<float>(),
                        *action_dst = action_buffer[start_index].data<float>(),
                        *advants_dst = advants_buffer[start_index].data<float>(),
                        *ret_dst = ret_buffer[start_index].data<float>();
                    memcpy(state_dst, state.data<float>(), cp_length * state_length * sizeof(float) );
                    memcpy(action_dst, action.data<float>(), cp_length * action_length * sizeof(float));
                    memcpy(advants_dst, advantage.data<float>(), cp_length* sizeof(float));
                    memcpy(ret_dst, ret.data<float>(), cp_length * sizeof(float));
                    Clear();
                    return start_index + cp_length;
                }
                return -1;
            }

            int count;
            torch::Tensor state, action, reward, advantage, ret;
            torch::Device cuda_device, cpu_device;
            float kGamma, kLambda;
            float *state_ptr, *action_ptr, *reward_ptr;
            int state_length, action_length;

        };

        class TrajectoryBuffer{
        public:
            TrajectoryBuffer(torch::Device &d, int state_length, int action_length, float kGamma, float kLambda, int agent_number = 1,
                    int max_points_length = 4096, int max_trajectory_length = 802) :
                    count(0), max_length(max_points_length), cpu_device(torch::kCPU),
                    kActionLength(action_length), kStateLength(state_length){
                buffer = std::vector<TrajectoryPoints>(agent_number, TrajectoryPoints(d, state_length,
                        action_length, max_trajectory_length, kGamma, kLambda));
                state_buffer = torch::zeros({max_points_length, state_length}, cpu_device).detach();
                action_buffer = torch::zeros({max_points_length, action_length}, cpu_device).detach();
                advants_buffer = torch::zeros({max_points_length, 1}, cpu_device).detach();
                ret_buffer = torch::zeros({max_points_length, 1}, cpu_device).detach();
            }

            void Clear(){
                count = 0;
                for (auto &item : buffer){
                    item.Clear();
                }
            }

            bool AddData(std::vector<std::vector<float> >&state, std::vector<std::vector<float> >&action,
                    std::vector<float> &reward, std::vector<float> &done, std::shared_ptr<CriticModel> &model
                    ){
                for (int i = 0; i < state.size(); i++){
                    int t = buffer[i].AddData(state[i], action[i], reward[i], done[i],
                            model, state_buffer, action_buffer, advants_buffer, ret_buffer, max_length, count);
                    if (t != -1){
                        count = t;
                    }
                    if (count >= max_length){
                        break;
                    }
                }
                if (count >= max_length){
                    Clear();
                    return true;
                }
                return false;
            }

            bool AddData(std::vector<cs::CarTransition> &data, std::shared_ptr<CriticModel> &model){
                for (int i = 0; i < data.size(); ++i){
                    int t = buffer[i].AddData(data[i].state, data[i].action, data[i].reward, data[i].done,
                            model, state_buffer, action_buffer, advants_buffer, ret_buffer, max_length, count);
                    if (t != -1){
                        count = t;
                    }
                    if (count >= max_length){
                        break;
                    }
                }
                if (count >= max_length){
                    Clear();
                    return true;
                }
                return false;
            }

            void Sample(std::vector<int> &index_buffer, int start, int len,
                    torch::Tensor &sample_state, torch::Tensor &sample_action, torch::Tensor & sample_advants,
                    torch::Tensor &sample_return){
                float *src_state = state_buffer.data<float>(),
                        *src_action = action_buffer.data<float>(),
                        *src_ret = ret_buffer.data<float>(),
                        *src_advants = advants_buffer.data<float>(),
                        *dst_state = sample_state.data<float>(),
                        *dst_action = sample_action.data<float>(),
                        *dst_return = sample_return.data<float>(),
                        *dst_advants = sample_advants.data<float>();

                for (int i = start ;i < start + len ; i++){
                    memcpy(dst_action, src_action + index_buffer[i] * kActionLength, sizeof(float) * kActionLength);
                    memcpy(dst_state, src_state + index_buffer[i] * kStateLength, sizeof(float) * kStateLength);

                    dst_action += kActionLength;
                    dst_state += kStateLength;

                    *(dst_return ++) = *(src_ret + index_buffer[i]);
                    *(dst_advants ++) = *(src_advants + index_buffer[i]);
                }
            }



            ~TrajectoryBuffer() = default;

            std::vector<TrajectoryPoints> buffer;

            int count, max_length, kActionLength, kStateLength;

            torch::Tensor state_buffer, action_buffer, advants_buffer, ret_buffer;

            torch::Device cpu_device;
        };
    }
}
#endif //TRAJECTORY_BUFFER_H
