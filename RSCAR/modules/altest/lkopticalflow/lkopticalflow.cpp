/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : lkopticalflow.cpp
*   Author      : FanmingL
*   Created date: 2018-10-18 13:58:51
*   Description : 
*
*===============================================================*/


#include "lkopticalflow.h"
namespace rs{
    namespace vp{

        void LkOpticalFlow::PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) {
            if (need_init){
                dst = src.clone();
                need_init = false;
            }
            else{
               cv::Mat res, res8u;
                cv::optflow::calcOpticalFlowSparseToDense(last, src, res);
                cv::Mat resabs(res.rows, res.cols, CV_32FC1, cv::Scalar(0));
                for (int row = 0; row < res.rows; row++){
                    for (int col = 0; col < res.cols; col++){
                        auto item = res.at<cv::Vec2f>(row, col);
                        resabs.at<float>(row, col) = sqrtf(item[0] * item[0] + item[1] *item[1]);
                        if (resabs.at<float>(row, col) > lk_config.threshold_diff())resabs.at<float>(row,col) = lk_config.threshold_diff();
                    }
                }
                cv::normalize(resabs,res8u,0,255,CV_MINMAX);
                cv::Mat obj_mask;
                cv::convertScaleAbs(res8u, obj_mask);
                obj_mask = obj_mask > lk_config.threshold_binary();
                dst = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
                src.copyTo(dst, obj_mask);
            }


            last = src.clone();
        }

        LkOpticalFlow::LkOpticalFlow() : need_init(true){
            ReadConfig();
            lk_config.PrintDebugString();
        }

        void LkOpticalFlow::ReadConfig() {
            common::ReadProtoFromTextFile("modules/altest/lkopticalflow/config/lkopticalflow.prototxt", &lk_config);
        }
    }
}
