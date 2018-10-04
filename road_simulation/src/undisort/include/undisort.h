//
// Created by erdou on 18-10-3.
//
#include <iostream>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>

#ifndef ROAD_SIMULATION_UNDISORT_H
#define ROAD_SIMULATION_UNDISORT_H


class undisort {

public:
    undisort(const std::string &data_in_path, const std::string &data_out_path);

    bool run(cv::Mat &dst);

    const std::string base_path = std::string(PATH);
    cv::Mat inner_matrix;
    cv::Mat disorted_matrix;
    std::vector<double> disorted_vector;
private:
    cv::VideoCapture videoCapture;
    cv::VideoWriter videoWriter;

};


#endif //ROAD_SIMULATION_UNDISORT_H
