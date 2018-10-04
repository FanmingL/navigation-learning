//
// Created by erdou on 18-10-3.
//

#include <src/undisort/include/undisort.h>

#include "undisort.h"

undisort::undisort(const std::string &data_in_path, const std::string &data_out_path) {
    std::string config_path = base_path + "/data/camera.xml";
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    cv::FileNode root_node = fs.root();
    auto iter = root_node.begin();
    (*iter) >> inner_matrix;
    iter++;
    (*iter) >> disorted_matrix;
    disorted_vector.push_back(disorted_matrix.at<double>(0, 0));
    disorted_vector.push_back(disorted_matrix.at<double>(0, 1));
    disorted_vector.push_back(0);
    disorted_vector.push_back(0);
    //std::cout<<inner_matrix<<std::endl;
    //std::cout<<disorted_matrix<<std::endl;
    std::string data_in_path_real = base_path + data_in_path, data_out_path_real = base_path + data_out_path;
    videoCapture.open(data_in_path_real);
    std::remove((char *) data_out_path_real.c_str());
    videoWriter.open(data_out_path_real, CV_FOURCC('D', 'I', 'V', 'X'), 30, cv::Size(960, 960));
}

bool undisort::run(cv::Mat &dst) {
    cv::Mat m, tmp;
    cv::Rect rect(1280 - 960, 0, 960, 960);
    videoCapture >> m;
    if (m.empty())return false;
    cv::undistort(m, tmp, inner_matrix, disorted_vector);
    dst = tmp(rect).clone();
    videoWriter << dst;
    return true;
}





