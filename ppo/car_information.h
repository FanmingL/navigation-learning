//
// Created by Fanming Luo on 2019/1/12.
//

#ifndef CAR_SIMULATOR_CAR_INFORMATION_H
#define CAR_SIMULATOR_CAR_INFORMATION_H

#include "util.h"
#include <vector>
#include <ostream>

namespace rs {
    namespace cs {
        class CarMemory {
        public:
            friend std::ostream &operator<<(std::ostream &os, const CarMemory &memory) {
                os << "state: ";
                for (auto &item : memory.state)
                    os << item << ", ";
                os << std::endl;
                os << " action: ";
                for (auto &item:memory.action)
                    os << item << ", ";
                os << std::endl;
                os << " reward: " << memory.reward
                   << " mask: " << memory.mask;
                return os;
            }

            std::vector<std::vector<float> > GetAsVector() {
                std::vector<std::vector<float> > res(4);
                res[0] = std::move(state);
                res[1] = std::move(action);
                res[2].push_back(reward);
                res[3].push_back(mask);
                return std::move(res);
            }

            CarMemory(std::vector<float> &state_, std::vector<float> &action_, const float &reward_, const int &mask_) :
                    state(std::move(state_)), action(std::move(action_)), reward(reward_), mask(mask_) {}

            std::vector<float> state;
            std::vector<float> action;
            float reward;
            float mask;
        };

        class CarInformation {
        public:
            CarInformation(const float &x_, const float &y_, const float &vx_, const float &vy_,
                           const float &target_x_, const float &target_y_, const int &index_) :
                    x(x_), y(y_), vx(vx_), vy(vy_), target_x(target_x_), target_y(target_y_), index(index_) {
                init_distance_to_target = MyNorm(x - target_x, y - target_y) + 1;
                angle = atan2f(vy, vx);
                mask = 1;
                reward = 0;
                return_value = 0;
                count = 0;
            }

            float x, y, vx, vy, target_x, target_y, init_distance_to_target, angle;
            float return_value;
            int index;
            int count;

            std::vector<float> state;
            std::vector<float> action;
            float reward;
            /* mask == 0 only at the end of a trajectory! */
            int mask;
            std::vector<CarMemory> memory;
        };
    }
}

#endif //CAR_SIMULATOR_CAR_INFORMATION_H
