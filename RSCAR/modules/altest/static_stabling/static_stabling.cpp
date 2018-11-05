//
// Created by erdou on 18-11-4.
//

#include "static_stabling.h"


namespace rs{
    namespace vp{


        static_stabling::static_stabling() : init_flag(false),
                 feature_detector(cv::xfeatures2d::SiftFeatureDetector::create()),
                 descriptor(cv::xfeatures2d::SiftDescriptorExtractor::create()),
                 homograph_matrix(3,3,CV_64FC1, cv::Scalar(0))
        {
            ReadConfig();
#ifdef USE_KNN
            bgsubtractor = cv::createBackgroundSubtractorKNN();
#else
            bgsubtractor = cv::createBackgroundSubtractorMOG2();
            bgsubtractor->setVarThreshold(50);
#endif
            bgsubtractor->setDetectShadows(true);
            bgsubtractor->setHistory(500);
            bgsubtractor->setShadowThreshold(0.4);
            bgsubtractor->setShadowValue(50);
        }
        /// enter!!!!!!!!!
        void static_stabling::PerformAlgorithm(const cv::Mat &src, cv::Mat &dst) {
            if (if_need_init())init(src);
            std::vector<cv::Rect2f> bbox;
            std::vector<std::vector<cv::Point2f> > interesting_points;
            cv::Mat gray;
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
            /// background
            /// corner points
            GetPointsPair(interesting_points, gray, roi);
            if (!init_points.empty()){
                Match(interesting_points);
            }else{
                init_points = interesting_points;
            }

            dst = src.clone();
            cv::warpPerspective(dst, dst, homograph_matrix, dst.size());
            dst = dst(show_roi);
            bgsubtractor->apply(dst, bgmask, -1);
            MyRefine(bgmask, 2, bbox);
            for (auto &item : bbox)cv::rectangle(dst, item, cv::Scalar(0,0,255),2);
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
           show_roi = cv::Rect(cv::Point(110,252), cv::Point(1000,1000));
           init_flag = true;
           homograph_matrix.at<double>(0,0) = homograph_matrix.at<double>(1,1) =homograph_matrix.at<double>(2,2)  = 1;
           std::cout<<"INIT DONE\n";
        }

        void static_stabling::MyRefine(cv::Mat &mask, int radius,std::vector<cv::Rect2f> &bboxs) {
            mask = (mask > 130);

            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius, radius));
            cv::morphologyEx(mask,mask,cv::MORPH_OPEN, element);
            element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size( 2 * radius, 2 *   radius));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
            //cv::dilate(mask,mask,element);
            //element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius,  radius));
            //cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
            cv::Mat mask_copy = mask.clone();
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask_copy, contours, hierarchy,cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
            for (auto &item : contours){
                double area = cv::contourArea(item);
                if (area < 110)continue;
                cv::Rect2f rect= cv::boundingRect(item);
                //if (rect.width / rect.height > 5 || rect.width / rect.height < 0.2)continue;

                cv::Point2f tl, br;
                //common::CalculateTransform(homograph_matrix.inv(), rect.tl(), tl, common::DOUBLE_TYPE);
                //common::CalculateTransform(homograph_matrix.inv(), rect.br(), br, common::DOUBLE_TYPE);
                //bboxs.emplace_back(tl,br);
                bboxs.push_back(rect);
            }

        }

        /*q = x * cos theta + y * sin theta*/
        void static_stabling::MyHoughLine(const std::vector<cv::Point2f> &points, const int &threshold, std::vector<cv::Vec2f> &lines) {
            int den_q = 3;
            int theta_num = 360;
            int q_max = (int)(sqrt(max_rect.width * max_rect.width + max_rect.height * max_rect.height)/den_q);
            std::vector<std::vector<unsigned short > > count_map(theta_num, std::vector<unsigned short>(q_max, 0));
            for (const auto & item : points){
                for (int i = 0; i < theta_num; i++){
                    float theta = ((float) (i  * CV_PI)) / theta_num;
                    int q_tmp = (int) ((item.x * cosf(theta) + item.y * sinf(theta)) / den_q);
                    if (q_tmp < 0 || q_tmp >= q_max)continue;
                    count_map[i][q_tmp]++;
                }
            }
            std::vector<std::pair<int, cv::Vec2f> > pair_res;
            for (int i = 0; i < count_map.size(); i++){
                auto &item = count_map[i];
                for (int j = 0; j < item.size(); j++){
                    if (item[j] > threshold){
                        pair_res.emplace_back(std::pair<int, cv::Vec2f>(item[j], cv::Vec2f(i * 180 / (float)theta_num,j*den_q)));
                    }
                }
            }
            std::sort(pair_res.begin(), pair_res.end(), [](const std::pair<int, cv::Vec2f>& v1, const std::pair<int, cv::Vec2f>& v2)->bool{return v1.first > v2.first;});
            std::vector<int> dis_buffer;
            for (auto &item :pair_res){
                bool flag = true;
                for (auto &item2 : dis_buffer){
                    if (std::abs(item2 - item.second[1]) < 70 * den_q){
                        flag = false;
                        break;
                    }
                }
                if (flag) {
                    lines.push_back(item.second);
                    dis_buffer.push_back((int)item.second[1]);
                }
            }


        }

        void static_stabling::GetPointsPair(std::vector<std::vector<cv::Point2f> > &points, const cv::Mat &gray, const cv::Rect2f &roi, const cv::Mat &mask) {
            points.clear();
            points = std::vector<std::vector<cv::Point2f> >(2);
            std::vector<cv::Point2f> interest_points;
            cv::goodFeaturesToTrack(gray(roi), interest_points, 300, 0.24, 15, mask, 3, false, 0.04);
#if 0
            cv::Size winSize(3,3), zeroZone(-1,-1);
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 200, 0.001);
            cv::cornerSubPix(gray, interest_points, winSize, zeroZone, criteria);
#endif
            std::vector<cv::Vec2f> lines;
            MyHoughLine(interest_points, 8, lines);

            cv::Mat line_mask(max_rect.size(), CV_8UC1, cv::Scalar(0));
            for (int i = 0; i < 2 && i < lines.size(); i++){
                auto &item = lines[i];
                float cos_tmp = cosf(item[0] * (float)CV_PI / 180),
                        sin_tmp = sinf(item[0] * (float)CV_PI / 180),
                        x0 = item[1] * cos_tmp,
                        y0 = item[1] * sin_tmp,
                        //x1 = x0 - 2000 * -sin_tmp,
                        x1 = 0,
                        r1 = x0 / (- sin_tmp),
                        y1 = y0 - r1 * cos_tmp,
                        //x2 = x0 + 2000 * -sin_tmp,
                        x2 = max_rect.width,
                        r2 = (x2 - x0) / (-sin_tmp),
                        y2 = y0 + r2 * cos_tmp;
                //cv::line(dst,cv::Point2f(x1, y1) + roi2f.tl(), cv::Point2f(x2, y2) + roi2f.tl(), cv::Scalar(0, 255, 0),12);
                cv::line(line_mask,cv::Point2f(x1, y1) + roi2f.tl(), cv::Point2f(x2, y2) + roi2f.tl(), cv::Scalar((i+1) * 122),12);
            }
            if (lines.size() < 2)std::cout<<"111\n";
            for (auto &item : interest_points){
                item += roi2f.tl();
                if (line_mask.at<uint8_t>(item) == 122){
                    points[0].push_back(item);
                }else if (line_mask.at<uint8_t>(item) == 244){
                    points[1].push_back(item);
                }
            }
            if (points[0][0].y > points[1][0].y){
                auto tmp = points[0];
                points[0] = points[1];
                points[1] = tmp;
            }
            std::sort(points[0].begin(), points[0].end(), [](const cv::Point2f &p1, const cv::Point2f &p2)->bool{return p1.x > p2.x;});
            std::sort(points[1].begin(), points[1].end(), [](const cv::Point2f &p1, const cv::Point2f &p2)->bool{return p1.x > p2.x;});
        }

        void static_stabling::Match(const std::vector<std::vector<cv::Point2f> > &points) {
            std::vector<std::vector<cv::Point2f> > matched_points = std::vector<std::vector<cv::Point2f> >(2);
            auto points_cvt = points;
            for (auto &item : points_cvt){
                for (auto &item2 : item){
                    common::CalculateTransform(homograph_matrix, item2, item2, common::DOUBLE_TYPE);
                }
            }
            for (int i = 0; i < 2; ++i){
                for (int j = 0; j < points[i].size(); ++j){
                    bool flag = false;
                    cv::Point2f res;
                    for (const auto &item2 : init_points[i]){
                        auto item3 = points_cvt[i][j] - item2;
                        if (sqrtf(item3.x * item3.x + item3.y * item3.y) < 4){
                            flag = true;
                            res = item2;
                            break;
                        }
                    }
                    if (flag){
                        matched_points[0].push_back(points[i][j]);
                        matched_points[1].push_back(res);
                    }
                }
            }
            cv::Mat out_mask;
            homograph_matrix = cv::findHomography(matched_points[0], matched_points[1], cv::RANSAC, 4, out_mask);
        }
    }
}
