//
// Created by Fanming Luo on 2019/1/17.
//

#ifndef CAR_SIMULATOR_MODEL_H
#define CAR_SIMULATOR_MODEL_H

#include <torch/torch.h>
#include <memory>
#include "model_base.h"

namespace rs {
    namespace al {
        class ActorModel : public ModelBase {
        public:
            ActorModel(int feature_length, const torch::Device &d, const std::string &_name) : ModelBase(d, _name) {
                int hidden1 = 400, hidden2 = 300, hidden3 = 40;
                int self_feature = 100, env_feature = 800;
                fc.emplace_back(register_module("self_feature", torch::nn::Linear(4, self_feature)));
                fc.emplace_back(register_module("env_feature", torch::nn::Linear(feature_length - 4, env_feature)));
                fc.emplace_back(register_module("hidden1", torch::nn::Linear(self_feature + env_feature, hidden1)));
                fc.emplace_back(register_module("hidden2", torch::nn::Linear(hidden1, hidden2)));
                fc.emplace_back(register_module("hidden3", torch::nn::Linear(hidden2, hidden3)));
                fc.emplace_back(register_module("out", torch::nn::Linear(hidden3, 2)));

                for (auto &item :fc) {
                    item->weight.set_requires_grad(false);
                    item->weight = item->weight.uniform_(0, 1e-4);
                    item->weight.set_requires_grad(true);
                }
            }

            ~ActorModel() override = default;

            torch::Tensor forward(torch::Tensor data) {
                auto self_feature = data.index_select(1, torch::arange(0, 4).toType(torch::kInt64).to(device));
                auto env_feature = data.index_select(1,
                                                     torch::arange(4, data.size(1)).toType(torch::kInt64).to(device));
                self_feature = torch::relu(fc[0]->forward(self_feature));
                env_feature = torch::relu(fc[1]->forward(env_feature));
                data = torch::cat({self_feature, env_feature}, 1);
                data = torch::relu(fc[2]->forward(data));
                data = torch::relu(fc[3]->forward(data));
                data = torch::relu(fc[4]->forward(data));
                data = torch::tanh(fc[5]->forward(data));
                return std::move(data);
            }
        };

        class CriticModel : public ModelBase {
        public:
            CriticModel(const int &feature_length, const torch::Device &d, const std::string &_name) :
                    ModelBase(d, _name) {
                int hidden_self_feature = 100, hidden_self_env_feature = 800;
                int hidden1 = 400, hidden2 = 300, hidden3 = 40;
                fc.emplace_back(register_module("self_feature", torch::nn::Linear(4, hidden_self_feature)));
                fc.emplace_back(
                        register_module("env_feature", torch::nn::Linear(feature_length - 4, hidden_self_env_feature)));
                fc.emplace_back(register_module("hidden1", torch::nn::Linear(
                        hidden_self_feature + hidden_self_env_feature, hidden1)));
                fc.emplace_back(register_module("hidden2", torch::nn::Linear(hidden1, hidden2)));
                fc.emplace_back(register_module("hidden3", torch::nn::Linear(hidden2, hidden3)));
                fc.emplace_back(register_module("out", torch::nn::Linear(hidden3, 1)));
                for (auto &item :fc) {
                    item->weight.set_requires_grad(false);
                    item->weight = item->weight.uniform_(0, 1e-5);
                    item->weight.set_requires_grad(true);
                }
            }

            ~CriticModel() override = default;

            /*  state
             *  action
             * */
            torch::Tensor forward(torch::Tensor data) {
                auto self_feature = data.index_select(1, torch::arange(0, 4).toType(torch::kInt64).to(device));
                auto env_feature = data.index_select(1,
                                                     torch::arange(4, data.size(1)).toType(torch::kInt64).to(device));

                self_feature = torch::relu(fc[0]->forward(self_feature));
                env_feature = torch::relu(fc[1]->forward(env_feature));
                data = torch::cat({self_feature, env_feature}, 1);
                data = torch::relu(fc[2]->forward(data));
                data = torch::relu(fc[3]->forward(data));
                data = torch::relu(fc[4]->forward(data));
                data = fc[5]->forward(data);
                return std::move(data);
            }

        };
    }
}
#endif //CAR_SIMULATOR_MODEL_H
