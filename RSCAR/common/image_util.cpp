//
// Created by erdou on 18-11-4.
//
#include "common/image_util.h"

namespace rs {
    namespace common {

        void GetOneChannel(const cv::Mat &src, cv::Mat &dst, int channel_index) {
            std::vector<cv::Mat> hsv_vector;
            cv::split(src, hsv_vector);
            dst = hsv_vector[channel_index].clone();
        }

        void GetHue(const cv::Mat &src, cv::Mat &dst, int channel_index) {
            cv::Mat hsv;
            cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV_FULL);
            GetOneChannel(hsv, dst, channel_index);
        }

        void CalculateTransform(const cv::Mat &transform_matrix, const cv::Point2f &src_point, cv::Point2f &dst_point,
                                int type) {
            if (type == 1) {
                cv::Mat cal_tmp(3, 1, CV_32FC1, cv::Scalar(1)), res_tmp(3, 1, CV_32FC1, cv::Scalar(1));
                cal_tmp.at<float>(0, 0) = src_point.x;
                cal_tmp.at<float>(0, 1) = src_point.y;
                res_tmp = transform_matrix * cal_tmp;
                res_tmp /= res_tmp.at<float>(2, 0);
                dst_point.x = res_tmp.at<float>(0, 0);
                dst_point.y = res_tmp.at<float>(1, 0);
            } else {
                cv::Mat cal_tmp(3, 1, CV_64FC1, cv::Scalar(1)), res_tmp(3, 1, CV_64FC1, cv::Scalar(1));
                cal_tmp.at<double>(0, 0) = (double) src_point.x;
                cal_tmp.at<double>(0, 1) = (double) src_point.y;
                res_tmp = transform_matrix * cal_tmp;
                res_tmp /= res_tmp.at<double>(2, 0);
                dst_point.x = (float) res_tmp.at<double>(0, 0);
                dst_point.y = (float) res_tmp.at<double>(1, 0);
            }
        }

        float CalculateRectOverlapRatio(const cv::Rect2f &r1, const cv::Rect2f &r2, int mode) {
            float a1 = r1.area(), a2 = r2.area(), a3 = (r1 & r2).area();
            if (mode == 1) {
                return (a3 / (a1 + a2 - a3));
            } else if (mode == 2) {
                return a3 / std::min(a1, a2);
            } else if (mode == 3) {
                return a3 / std::max(a1, a2);
            }
            return 0;
        }

    }
}