//
// Created by Fanming Luo on 2018/9/23.
//

#ifndef YOLO_YOLO_DARKNET_H
#define YOLO_YOLO_DARKNET_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#ifdef __cplusplus
extern "C" {
#endif
#include "darknet.h"
    image ipl_to_image(IplImage *src);
    IplImage* image_to_ipl(image im);
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

class YOLO_DARKNET {

public:
    class YOLO_OUT{
        friend std::ostream & operator<<(std::ostream &out, YOLO_OUT &obj);
    public:
        float prob;
        std::string name;
        box bbox;
        YOLO_OUT(const char *_name, box &_bbox, float _prob):
        name(_name), bbox(_bbox), prob(_prob){}

    };
    YOLO_DARKNET();
    ~YOLO_DARKNET() = default;
    void yoloProcess(cv::Mat &in, cv::Mat &res, std::vector<YOLO_OUT> &anchor, float threshold, float hier, float nms);
    void printDetection(std::vector<YOLO_OUT> &detection_ins);
    std::string basePath, configPath, weightPath, imagePath,
            namesPath, windowName, alphabetPath;
    unsigned int counter;

private:
    void load_alphabet_new();
    void draw_detections_new(image im, detection *dets, int num,
            float thresh, int classes, std::vector<YOLO_OUT> &res_name);
    image **alphabets;
    char **names;
    network *net;


};


#endif //YOLO_YOLO_DARKNET_H
