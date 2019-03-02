/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : ppo_main.cpp
*   Author      : FanmingL
*   Created date: 2019-01-26 20:26:56
*   Description : 
*
*===============================================================*/
#include "car_simulator.h"
#include <random>
#include <chrono>
#include "unistd.h"
#include "trajectory_buffer.h"
#include "ppo.h"
int main(int argc, char **argv){
    int count = 0;
    rs::al::PPO ppo;
    auto t1 = std::chrono::high_resolution_clock::now();
    float average_return, actor_loss, critic_loss;
    while (true){
        average_return = ppo.TrainOneEpoch(actor_loss, critic_loss);
        printf("%8d, %12.7f, %12.7f, %12.7f\n", count, average_return, actor_loss, critic_loss);
        count ++;
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout<<"fps: "<<1000000000.f/(t2-t1).count()<<std::endl;
        t1 = std::chrono::high_resolution_clock::now();
        if (count % 50 == 0){
            ppo.SaveModel( count / 50);
        }
    }


    return 0;
}

