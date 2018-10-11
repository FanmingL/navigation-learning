//
// Created by erdou on 18-10-11.
//

#include "BFSeparation.h"
namespace rs{
    namespace vp{

        BFSeparation::BFSeparation() {
            SetAlgorithmName("background and foreground algorithm");
        }

        void BFSeparation::DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res) {
            res.clear();
            output = input.clone();
        }
    }
}