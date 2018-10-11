//
// Created by erdou on 18-10-11.
//

#ifndef RSCAR_YOLODETECTION_H
#define RSCAR_YOLODETECTION_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgcodecs.hpp"
#include "darknet.h"

#ifdef __cplusplus
extern "C" {
#endif
image ipl_to_image(IplImage *src);
IplImage *image_to_ipl(image im);
cv::Mat image_to_mat(image im);
image mat_to_image(cv::Mat m);
float get_color(int, int, int);
void embed_image(image source, image dest, int dx, int dy);
#ifdef __cplusplus
}
#endif

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include "modules/detect/detect.pb.h"
#include "modules/detect/Yolo/YoloConfig.pb.h"
#include "modules/detect/detect_algorithm_base.h"
#include "common/io.h"
#include "common/algorithm_factory.h"

namespace rs {
    namespace vp {
        class YoloDetection : public DetectAlgorithmBase{

        public:
            YoloDetection();

            ~YoloDetection() override = default;

            void DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res) override;

            void ReadConfig();
            //void yoloProcess(cv::Mat &in, cv::Mat &res, std::vector<YOLO_OUT> &anchor, float threshold, float hier,
            //                 float nms);

        private:
            void load_alphabet_new();

            YoloConfig yolo_config;
            void draw_detections_new(image im, detection *dets, int num,
                                     float thresh, int classes, std::vector<DetectData> &res_name);

            image **alphabets;
            char **names;
            network *net;
        };
        rs::common::REGISTER_ALGORITHM(DetectAlgorithmBase,"Yolo", YoloDetection);
    }
}



#endif //RSCAR_YOLODETECTION_H
