//
// Created by erdou on 18-11-4.
//

#include "static_stabling.h"


namespace rs{
    namespace vp{


        static_stabling::static_stabling() : init_flag(false){
            ReadConfig();
            bgsubtractor = cv::createBackgroundSubtractorMOG2();
            bgsubtractor->setVarThreshold(40);
            bgsubtractor->setDetectShadows(true);
            bgsubtractor->setHistory(500);
            bgsubtractor->setShadowThreshold(0.4);
        }

        void static_stabling::PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) {
            if (if_need_init()){
                init(src);
            }
            std::vector<cv::Point2f> interest_points;
            std::vector<cv::Rect2f> bbox;
            cv::Mat gray;
            cv::cvtColor(src(roi), gray, cv::COLOR_BGR2GRAY);
            bgsubtractor->apply(src, bgmask, -1);
            MyRefine(bgmask, 3, bbox);
            cv::goodFeaturesToTrack(gray, interest_points, 300, 0.23, 5, cv::Mat(), 3, false, 0.04);
            cv::Size winSize(3,3), zeroZone(-1,-1);
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 200, 0.001);
            cv::cornerSubPix(gray, interest_points, winSize, zeroZone, criteria);
            dst = src.clone();
            for (auto &item : interest_points){
                item += roi2f.tl();
                cv::circle(dst, item, 3, cv::Scalar(0,0,255), -1);
            }
            for (auto &item : bbox){
                cv::rectangle(dst, item, cv::Scalar(255,0,0),2);
            }
            cv::imshow("debug", bgmask);
        }

        void static_stabling::ReadConfig() {
            common::ReadProtoFromTextFile("modules/altest/static_stabling/config/static_stabling.prototxt", &config);
        }

        bool static_stabling::if_need_init() {
            return !init_flag;
        }

        void static_stabling::init(const cv::Mat &src) {
           if (!if_need_init())
               return;
           max_rect = cv::Rect(0,0,src.cols, src.rows);
           mask = cv::Mat(max_rect.size(), CV_8UC1, cv::Scalar(255));
           roi2f = roi = cv::Rect(0, 379, 1080, 500);
           init_flag = true;


           std::cout<<"INIT DONE\n";
        }

        void static_stabling::MyRefine(cv::Mat &mask, int radius,std::vector<cv::Rect2f> &bboxs) {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius, radius));
            cv::morphologyEx(mask,mask,cv::MORPH_OPEN, element);
            element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius*2, radius*2));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
            mask = (mask > 130);
            cv::Mat mask_copy = mask.clone();
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask_copy, contours, hierarchy,cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
            for (auto &item : contours){
                double area = cv::contourArea(item);
                if (area < 100)continue;
                cv::Rect2f rect= cv::boundingRect(item);
                if (rect.width / rect.height > 5 || rect.width / rect.height < 0.2)continue;
                bboxs.push_back(rect);
            }

        }
    }
}
