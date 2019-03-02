/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : main.cpp
*   Author      : FanmingL
*   Created date: 2019-01-12 11:42:20
*   Description : 
*
*===============================================================*/
#include "car_simulator.h"
#include <random>
#include <chrono>
#include "unistd.h"
int main(int argc, char **argv){
    rs::cs::CarSimulator car_simulator;
    car_simulator.Reset();
    int count = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    while (true){
        //car_simulator.RunWithoutParam();
        car_simulator.GetFromRealData(count % 15000);
        count ++;
        if (count % 1000 == 0){
            auto t2 = std::chrono::high_resolution_clock::now();
            std::cout<<1000000000000.f/(t2-t1).count()<<std::endl;
            t1 = std::chrono::high_resolution_clock::now();
        }
    }


	return 0;
}
