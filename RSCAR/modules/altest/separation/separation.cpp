/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : separation.cpp
*   Author      : FanmingL
*   Created date: 2018-10-18 22:14:43
*   Description : 
*
*===============================================================*/


#include "separation.h"


namespace rs{
    namespace vp{

        separation::separation() {
            ReadConfig();
            bgsubtractor = cv::createBackgroundSubtractorMOG2();
            bgsubtractor->setVarThreshold(separation_config.threshold());
            bgsubtractor->setDetectShadows(separation_config.if_detect_shadow());
            bgsubtractor->setHistory(separation_config.history_number());
            bgsubtractor->setShadowThreshold(separation_config.shadow_threshold());
        }

        void separation::PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) {
            cv::Mat front_mask;
            dst = src.clone();
            bgsubtractor->apply(src, bgmask, separation_config.learning_rate());
            //RefineSegments(src, bgmask, front_mask);
            MyRefine(bgmask, separation_config.open_radius());
            //dst = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));
            //src.copyTo(dst,bgmask);
            dst = src.clone();
            cv::imshow("mask", bgmask);
        }

        void separation::ReadConfig() {
            common::ReadProtoFromTextFile("modules/altest/separation/config/separation.prototxt", &separation_config);
            separation_config.PrintDebugString();
        }

        void separation::RefineSegments(const cv::Mat &src, cv::Mat &mask, cv::Mat &dst) {
            int niters = 3;

            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;

            cv::Mat temp;

            dilate(mask, temp, cv::Mat(), cv::Point(-1,-1), niters);
            erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
            dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

            findContours( temp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

            dst = cv::Mat::zeros(src.size(), CV_8UC1);

            if( contours.size() == 0 )
                return;

            // iterate through all the top-level contours,
            // draw each connected component with its own random color
            int idx = 0, largestComp = 0;
            double maxArea = 0;

            for( ; idx >= 0; idx = hierarchy[idx][0] )
            {
                const std::vector<cv::Point>& c = contours[idx];
                double area = fabs(contourArea(cv::Mat(c)));
                if( area > maxArea )
                {
                    maxArea = area;
                    largestComp = idx;
                }
            }
            cv::Scalar color(255 );
            drawContours( dst, contours, largestComp, color, cv::FILLED, cv::LINE_8, hierarchy );
        }

        void separation::MyRefine(cv::Mat &mask, int radius) {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius, radius));
            cv::morphologyEx(mask,mask,cv::MORPH_OPEN, element);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
            mask = (mask > 130);

        }
    }
}

