//
// Created by erdou on 18-10-11.
//

#ifndef RSCAR_BFSEPARATION_H
#define RSCAR_BFSEPARATION_H

#include "modules/detect/detect_algorithm_base.h"
#include <string>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "modules/detect/BFSeparation/BFSeparation.pb.h"
#include "modules/detect/detect.pb.h"
#include "common/io.h"
#include "common/algorithm_factory.h"

namespace rs{
    namespace vp{
        class BFSeparation : public DetectAlgorithmBase{
        public:
            BFSeparation();
            ~BFSeparation() override = default;
            void DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res) override;
        private:

        };

        rs::common::REGISTER_ALGORITHM(DetectAlgorithmBase,"BFSeparation",BFSeparation);
    }
}


#endif //RSCAR_BFSEPARATION_H
