//
// Created by erdou on 18-10-18.
//

#ifndef RSCAR_ALTEST_ALGORITHM_BASE_H
#define RSCAR_ALTEST_ALGORITHM_BASE_H

#include <string>
#include "opencv2/core.hpp"
namespace rs{
    namespace vp{
        class AltestAlgorithmBase{
        public:
            AltestAlgorithmBase(){

            }

            virtual ~AltestAlgorithmBase() {}
            virtual void PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) = 0;
        private:
            std::string name;
        };
    }
}

#endif //RSCAR_ALTEST_ALGORITHM_BASE_H
