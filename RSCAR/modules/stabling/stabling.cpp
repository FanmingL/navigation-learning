/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : stabling.cpp
*   Author      : FanmingL
*   Created date: 2018-10-30 11:56:09
*   Description : 
*
*===============================================================*/


#include "stabling.h"


namespace rs{
    namespace vp{

        stabling::stabling(const std::string &name) : rs(name) ,
#ifdef USE_SIFT
        feature_detector(cv::xfeatures2d::SiftFeatureDetector::create()),
        descriptor(cv::xfeatures2d::SiftDescriptorExtractor::create()),
          bfMatcher(cv::NORM_L2),
#else
        feature_detector(cv::ORB::create(3000)),
        descriptor(cv::xfeatures2d::LATCH::create()),
          bfMatcher( cv::NORM_HAMMING ),
#endif
        count(0)
        {
            ReadConfig();
            video_capture.open(common::GetAbsolutePath(stabling_config.in_video_path()));
            max_rect = cv::Rect2d(0,0,
                    video_capture.get(CV_CAP_PROP_FRAME_WIDTH), video_capture.get(CV_CAP_PROP_FRAME_HEIGHT));
            resize_rect = cv::Rect(0,0,stabling_config.target_width(), stabling_config.target_height());

            if (stabling_config.if_write_video()){
                std::remove((char*)(common::GetAbsolutePath(stabling_config.out_video_path())).c_str());
                video_writer.open(common::GetAbsolutePath(stabling_config.out_video_path()), CV_FOURCC('D','I','V','X'),30,
                        max_rect.size());
            }
            label_mask = cv::imread(common::GetAbsolutePath(stabling_config.mask_path()), cv::IMREAD_GRAYSCALE);
            label_mask = label_mask >1;
            init();
        }

        void stabling::Run() {
            cv::Mat homograph;
            while (true){
                video_capture >> frame;
                auto frame_data = detect_video.frame(count);
                if (frame.empty())break;

                CalculateHomograph(frame, frame_data, homograph);
                canvas = frame.clone();
                cv::warpPerspective(canvas, canvas, homograph, canvas.size());

                if (stabling_config.if_write_video()){
                    video_writer << canvas;
                }
                if (stabling_config.if_show()){
                    cv::imshow("stabling", canvas);
                    if (cv::waitKey(1) == 'q')break;
                }
                AddData(homograph_data.add_matrix(), homograph);
                AddCount();
                for (int i = 0; i < stabling_config.step_count(); i++){
                    video_capture >> frame;
                    if (frame.empty())goto my_label;
                    AddData(homograph_data.add_matrix(), homograph);
                    AddCount();
                }
            }
            my_label:;
            common::WriteProtoToBinaryFile(stabling_config.out_data_path(), &homograph_data);
        }

        void stabling::ReadConfig() {
            common::ReadProtoFromTextFile("modules/stabling/config/stabling.prototxt", &stabling_config);
            common::ReadProtoFromBinaryFile(stabling_config.in_data_path(), &detect_video);
        }

        void stabling::AddCount() {
            std::cout<<count++<<std::endl;
        }

        cv::Rect stabling::CvtData(const DetectObject &object) {
            return cv::Rect((int)((object.x() - object.width()/2.0f) * max_rect.width),
                     (int)((object.y() - object.height()/2.0f) * max_rect.height),
                     (int)(object.width() * max_rect.width),
                     (int)(object.height() * max_rect.height));
        }

        void stabling::init() {
            cv::Mat gray;
            video_capture >> frame;
            cv::Mat mask(max_rect.size(), CV_8UC1, cv::Scalar(255));
            mask = label_mask.clone();
            for (auto &item : detect_video.frame(0).object()){
                auto rect_tmp = CvtData(item);
                rect_tmp.x -= stabling_config.padding_length();
                rect_tmp.y -= stabling_config.padding_length();
                rect_tmp.width += 2 * stabling_config.padding_length();
                rect_tmp.height += 2 * stabling_config.padding_length();
                mask(rect_tmp & max_rect) = cv::Scalar(0);
            }
            cv::resize(mask, mask, resize_rect.size(), 0, 0, cv::INTER_NEAREST);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::resize(gray, gray, resize_rect.size());
            last_frame = gray.clone();

            feature_detector->detect(gray, last_key_points, mask);
            descriptor->compute(gray, last_key_points, last_description);
            AddCount();
            last_homograph = cv::Mat(3,3,CV_64FC1,cv::Scalar(0));
            last_homograph.at<double>(0,0) = 1;
            last_homograph.at<double>(1,1) = 1;
            last_homograph.at<double>(2,2) = 1;
            //cv::drawKeypoints(gray, key_points, gray, cv::Scalar(0,0,255));
            //cv::imshow("init", gray);
            //cv::waitKey();
        }

        void stabling::CalculateHomograph(const cv::Mat &src, const DetectFrame &detect_frame, cv::Mat &dst) {
            cv::Mat gray, mask(max_rect.size(), CV_8UC1, cv::Scalar(255));
            mask = label_mask.clone();
            cv::warpPerspective(mask, mask, last_homograph.inv(), max_rect.size(), cv::INTER_NEAREST);
            cv::Mat tmp(max_rect.size(), CV_8UC3, cv::Scalar(0,0,0));
            src.copyTo(tmp, mask);
            cv::imshow("mask", tmp);
            std::vector<std::vector<cv::DMatch> > knn_matches;
            std::vector<cv::DMatch> good_matches;
            std::vector<cv::KeyPoint> key_points;
            cv::Mat description;
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
            for (auto &item : detect_frame.object()){
                cv::Rect tmp = CvtData(item) & max_rect;
                mask(tmp) = cv::Scalar(0);
            }
            cv::resize(mask, mask, cv::Size(stabling_config.target_width(), stabling_config.target_height()), 0, 0, cv::INTER_NEAREST);
            cv::resize(gray, gray, cv::Size(stabling_config.target_width(), stabling_config.target_height()));
            feature_detector->detect(gray, key_points, mask);
            descriptor->compute(gray, key_points, description);
            //bfMatcher.match(last_description, description, good_matches);
#if 1
            bfMatcher.knnMatch(last_description, description, knn_matches, 2);
            for (auto &item : knn_matches){
                if (item[0].distance / item[1].distance < stabling_config.ratio_threshold())
                    good_matches.push_back(item[0]);
            }
#else
            bfMatcher.match(last_description, description, good_matches);
#endif
            int match_num = stabling_config.point_num();
            /*
            if (good_matches.size() > match_num){
                std::nth_element(good_matches.begin(), good_matches.begin() + match_num - 1, good_matches.end());
                good_matches.erase(good_matches.begin() + match_num, good_matches.end());
            }
             */
            std::vector<cv::Point2f> last_points, now_points;
            for (auto &item : good_matches) {
                last_points.push_back(last_key_points[item.queryIdx].pt);
                now_points.push_back(key_points[item.trainIdx].pt);
            }
            cv::Mat out_mask;
            last_homograph = last_homograph * cv::findHomography(now_points, last_points, cv::RANSAC, stabling_config.ransac_threshold(), out_mask);
            std::cout<<last_homograph;
            cv::Mat canvas;
            cv::drawMatches(last_frame, last_key_points, gray, key_points, good_matches, canvas, cv::Scalar(0,0,255), cv::Scalar(255,0,0),out_mask);
            cv::imshow("match", canvas);
            last_description = description;
            last_key_points = key_points;
            last_frame = gray;
            dst = last_homograph.clone();
        }

        void stabling::AddData(HomographMatrix *data, const cv::Mat &homograph_matrix) {
            data->set_frame_index(count);
            data->set_h11((float)homograph_matrix.at<double>(0,0));
            data->set_h12((float)homograph_matrix.at<double>(0,1));
            data->set_h13((float)homograph_matrix.at<double>(0,2));
            data->set_h21((float)homograph_matrix.at<double>(1,0));
            data->set_h22((float)homograph_matrix.at<double>(1,1));
            data->set_h23((float)homograph_matrix.at<double>(1,2));
            data->set_h31((float)homograph_matrix.at<double>(2,0));
            data->set_h32((float)homograph_matrix.at<double>(2,1));
            data->set_h33((float)homograph_matrix.at<double>(2,2));
        }
    }
}

MAIN(rs::vp::stabling, "stabling");