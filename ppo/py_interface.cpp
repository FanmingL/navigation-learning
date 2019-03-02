/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : test.cpp
*   Author      : FanmingL
*   Created date: 2019-01-12 19:05:09
*   Description : 
*
*===============================================================*/


#include "ppo.h"
rs::al::PPO ppo;

extern "C" void Run(float *actor_loss, float *critic_loss, float *average){
    float a,c, re;
    re = ppo.TrainOneEpoch(a, c);
    *(actor_loss) = a;
    *(critic_loss) = c;
    *(average) = re;
}

extern "C" void Save(int index){
    ppo.SaveModel(index);
}

extern "C" void Load(int index){
    ppo.LoadModel(index);
}

extern "C" void Inference(float *ret){
	float return_value;
    ppo.RunWithoutSave(return_value, false);
    *(ret) = return_value;
}