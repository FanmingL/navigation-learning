/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : ppo_config.h
*   Author      : FanmingL
*   Created date: 2019-01-26 20:23:21
*   Description : 
*
*===============================================================*/


#ifndef _PPO_CONFIG_H
#define _PPO_CONFIG_H

#include <string>
namespace rs{
    namespace al{
        class PPOConfig{
        public:
            PPOConfig(
                const float &_kGamma = 1.f,
                const float &_kLambda = 0.98f,
                const float &_kClip = 0.2f,
                const float &_kNoise = 0.3f,
                const float &_kCriticLr = 0.0001f,
                const float &_kActorLr = 0.0001f,
                const int &_kBatchSize = 100,
                const int &_kPointsNum = 5000,
                const int &_kEpochNum = 10,
                const int &_kStateLength = 544,
                const int &_kActionLength = 2,
                const std::string &_out_path = "./out",
                const float &_kTargetKl = 0.01f,
                const int &_kRealDataNum = 1000,
                const float &_kEta = 0.005
                    )
                    :
                    kGamma(_kGamma),
                    kLambda(_kLambda),
                    kClip(_kClip),
                    kNoise(_kNoise),
                    kCriticLr(_kCriticLr),
                    kActorLr(_kActorLr),
                    kBatchSize(_kBatchSize),
                    kPointsNum(_kPointsNum),
                    kEpochNum(_kEpochNum),
                    out_path(_out_path),
                    kStateLength(_kStateLength),
                    kActionLength(_kActionLength),
                    kAgentNum(1),
                    kLogStd(-1),
                    kRealDataNum(_kRealDataNum),
                    kEta(_kEta)
                    {

            }

            ~PPOConfig() = default;

            float kActorLr, kCriticLr, kNoise, kClip, kLambda, kGamma, kLogStd, kTargetKl, kEta;

            int kBatchSize, kPointsNum, kEpochNum, kStateLength, kActionLength, kAgentNum, kRealDataNum;

            std::string out_path;

        };
    }
}
#endif //PPO_CONFIG_H
