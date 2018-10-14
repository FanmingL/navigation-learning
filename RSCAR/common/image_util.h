//
// Created by erdou on 18-10-12.
//

#ifndef RSCAR_IMAGE_UTIL_H
#define RSCAR_IMAGE_UTIL_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
namespace rs {
    namespace common {
        void GetHue(const cv::Mat &src, cv::Mat &dst, int channel_index = 0) {
            std::vector<cv::Mat> hsv_vector;
            cv::Mat hsv;
            cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV_FULL);
            //cv::cvtColor(src, hsv, cv::COLOR_BGR2YUV);
            cv::split(hsv, hsv_vector);
            dst = hsv_vector[channel_index].clone();
        }

        void CalculateTransform(const cv::Mat &transform_matrix, const cv::Point2f &src_point, cv::Point2f &dst_point)
        {
            cv::Mat cal_tmp(3,1,CV_32FC1,cv::Scalar(1)), res_tmp(3,1,CV_32FC1,cv::Scalar(1));
            cal_tmp.at<float>(0,0) = src_point.x;
            cal_tmp.at<float>(0,1) = dst_point.y;
            res_tmp = transform_matrix * cal_tmp;
            res_tmp /= res_tmp.at<float>(2,0);
            dst_point.x = res_tmp.at<float>(0,0);
            dst_point.y = res_tmp.at<float>(1,0);
        }
        enum OVERLAP_MODE{
            AND_OR = 1,
            AND_MIN = 2,
            AND_MAX = 3
        };
        float CalculateRectOverlapRatio(const cv::Rect2f &r1, const cv::Rect2f &r2, int mode = 2)
        {
            float a1 = r1.area(), a2 = r2.area();
            if (mode == 1) {
                return ((r1&r2).area() / (r1|r2).area());
            } else if (mode == 2) {
                return (r1 & r2).area() / std::min(a1, a2);
            }else if (mode == 3){
                return (r1 & r2).area() / std::max(a1, a2);
            }
        }
    }
}


#endif //RSCAR_IMAGE_UTIL_H
