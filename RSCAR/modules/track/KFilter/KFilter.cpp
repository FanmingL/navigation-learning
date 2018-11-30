//
// Created by erdou on 18-10-13.
//

#include "modules/track/KFilter/KFilter.h"
#include "KFilter.h"

namespace rs {

    namespace vp {

        KFilter::KFilter() {
            ReadConfig();
            SetAlgorithmName("Kalman Filter Tracking Algorhitm!!!");
            init_names();
            max_image_rect = cv::Rect2f(0, 0, kf_config.width(), kf_config.height());
            road_type_mask = cv::imread(common::GetAbsolutePath(kf_config.road_type_mask_path()),
                                        cv::IMREAD_GRAYSCALE);
            kf_config.PrintDebugString();
        }

        int single_tracker::car_index_counter = 0;

        void
        KFilter::Track(const cv::Mat &src, cv::Mat &dst, const DetectFrame &detect_frame, std::vector<TrackData> &res) {
            res.clear();
            std::vector<DetectData> detect_frame_vector;
            cv::Mat index_mat = ConvertData(detect_frame, detect_frame_vector);
            std::unordered_set<int> matched_index_set;
            for (auto iter = single_tracker_list.begin(); iter != single_tracker_list.end();) {
                TrackData tmp_track_data;
                //int matched_index = -1;
                std::vector<std::pair<float, int> > index_matched;
                if (iter->Update(src, detect_frame_vector, index_mat, tmp_track_data, index_matched)) {
                    bool flag = true;
                    for (auto &item : res) {
                        if (common::CalculateRectOverlapRatio(item.bbox, tmp_track_data.bbox, common::AND_OR) >
                            kf_config.max_overlap_ratio()) {

                            if (item.name == tmp_track_data.name) {
                                flag = false;
                                break;
                            } else if (peron_bicycle_motor.count(tmp_track_data.name) &&
                                       peron_bicycle_motor.count(item.name)) {
                                flag = false;
                                iter->last_track.name = "bicycle";
                                //item.name = "bicycle";
                                break;
                            } else if (car_truck_bus.count(item.name) && car_truck_bus.count(tmp_track_data.name)) {
                                flag = false;
                                break;
                            }
                        }
                    }
                    if (flag) {
                        res.push_back(tmp_track_data);
                        for (auto &item_index : index_matched)
                            matched_index_set.insert(item_index.second);
                        iter++;
                    } else {
                        for (auto &item_index : index_matched)
                            matched_index_set.insert(item_index.second);
                        iter = single_tracker_list.erase(iter);
                    }
                } else {
                    iter = single_tracker_list.erase(iter);
                }
            }
            std::vector<TrackData> new_in;
            for (int i = 0; i < detect_frame_vector.size(); i++) {
                if (matched_index_set.count(i))continue;
                bool flag = true;
                for (auto &item : new_in) {
                    if (common::CalculateRectOverlapRatio(item.bbox, detect_frame_vector[i].bbox) >
                        kf_config.min_overlap_ratio()) {
                        flag = false;
                        break;
                    }
                }
                if (flag) {
                    single_tracker single_tracker1(src, detect_frame_vector[i], kf_config);
                    single_tracker_list.emplace_back(single_tracker1);
                    new_in.push_back(single_tracker1.last_track);
                }
            }
            cv::Mat canvas = src.clone();
            for (auto &item : res) {
                if (item.probility != 0)
                    cv::rectangle(canvas, item.bbox, color_map[item.name], 2);
                else
                    cv::rectangle(canvas, item.bbox, cv::Scalar(0, 0, 0), 2);

                cv::putText(canvas, std::to_string(item.object_index), item.bbox.tl(),
                            cv::FONT_ITALIC, 0.8, cv::Scalar(0, 0, 128), 2);
            }
            dst = canvas;
        }

        void KFilter::ReadConfig() {
            common::ReadProtoFromTextFile("modules/track/KFilter/config/KFilter.prototxt", &kf_config);
        }

        bool KFilter::CheckConstraint(const DetectData &object) {
            float area = object.bbox.area();
            if (object.name == "person" && area > kf_config.person_max_area())return false;
            if (object.name != "car" && area > 2300)return false;
            if (object.name != "car" && ((!max_image_rect.contains(object.bbox.br() + cv::Point2f(15, 15))) ||
                                         (!max_image_rect.contains(object.bbox.tl() - cv::Point2f(15, 15)))))
                return false;
            if (peron_bicycle_motor.count(object.name)) {
                cv::Point2f p_tmp = object.bbox.br();
                p_tmp.x -= object.bbox.width / 2;
                if (max_image_rect.contains(p_tmp) && road_type_mask.at<uchar>(p_tmp) == common::CANNOT_GO)
                    return false;
            }
            return (area > kf_config.min_area() &&
                    area < kf_config.max_area() &&
                    object.probility > kf_config.probility_threshold() &&
                    names_should_take_care.count(object.name)
            );
        }

        cv::Rect2f KFilter::GetRect(const DetectObject &object) {
            return cv::Rect2f(object.x(), object.y(), object.width(), object.height());
            return cv::Rect2f((object.x() - object.width() / 2) * kf_config.width(),
                              (object.y() - object.height() / 2) * kf_config.height(),
                              object.width() * kf_config.width(), object.height() * kf_config.height());
        }

        void KFilter::init_names() {
            names_should_take_care.insert("car");
            color_map["car"] = cv::Scalar(0, 128, 0);
            names_should_take_care.insert("bus");
            names_should_take_care.insert("truck");
            names_should_take_care.insert("person");
            color_map["person"] = cv::Scalar(128, 0, 0);
            names_should_take_care.insert("bicycle");
            color_map["bicycle"] = cv::Scalar(128, 128, 0);
            names_should_take_care.insert("motorcycle");
            color_map["motorcycle"] = cv::Scalar(0, 128, 128);
            names_should_take_care.insert("motorbike");
            color_map["motorbike"] = cv::Scalar(0, 128, 128);
            peron_bicycle_motor.insert("person");
            peron_bicycle_motor.insert("bicycle");
            peron_bicycle_motor.insert("motorcycle");
            peron_bicycle_motor.insert("motorbike");
            car_truck_bus.insert("car");
            car_truck_bus.insert("truck");
            car_truck_bus.insert("bus");

        }

        cv::Mat KFilter::ConvertData(const DetectFrame &detect_frame, std::vector<DetectData> &detect_frame_vector) {
            detect_frame_vector.clear();
            cv::Mat index_mat(kf_config.height(), kf_config.width(), CV_32SC1, cv::Scalar(0));
            for (auto &item : detect_frame.object()) {
                auto _obj = DetectData(GetRect(item), item.name(), item.probility());
                if (!CheckConstraint(_obj)) continue;
                detect_frame_vector.emplace_back(_obj);
                cv::Point center = (_obj.bbox.tl() + _obj.bbox.br()) / 2;
                if (max_image_rect.contains(center)) {
                    int index_same_with = index_mat.at<int>(center);
                    if (index_same_with == 0) {
                        index_mat.at<int>(center) = (int) (detect_frame_vector.size() - 1);
                    } else {
                        if (detect_frame_vector[index_same_with].probility < _obj.probility) {
                            index_mat.at<int>(center) = (int) (detect_frame_vector.size() - 1);
                        }
                    }
                }
            }
            return index_mat;
        }

        single_tracker::single_tracker(const cv::Mat &src, const DetectData &_detect_data,
                                       const KFilterConfig &kf_config) {
            cv::Point2f tl = _detect_data.bbox.tl(), br = _detect_data.bbox.br();
            cv::TrackerKCF::Params param;
            param.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
            param.desc_npca = 0;
            param.compress_feature = true;
            param.compressed_size = 2;
            kcf_tracker = cv::TrackerKCF::create(param);
            kcf_tracker->init(src, _detect_data.bbox);
            mean_filter.emplace_back(common::mean_filter<cv::Point2f>(tl, kf_config.mean_filter_length()));
            mean_filter.emplace_back(common::mean_filter<cv::Point2f>(br, kf_config.mean_filter_length()));
            VECTOR_TYPE init_point;
            init_point[0] = tl.x;
            init_point[1] = tl.y;
            kalman_filter.emplace_back(KalmanFilter(init_point, MATRIX_I * kf_config.measure_cov(),
                                                    MATRIX_I * kf_config.control_cov()));
            init_point[0] = br.x;
            init_point[1] = br.y;
            kalman_filter.emplace_back(KalmanFilter(init_point, MATRIX_I * kf_config.measure_cov(),
                                                    MATRIX_I * kf_config.control_cov()));
            tracker_dog = std::make_shared<common::WatchDog>(common::WatchDog(kf_config.track_max_count()));
            yolo_dog = std::make_shared<common::WatchDog>(common::WatchDog(kf_config.yolo_max_count()));
            car_index_it = (++car_index_counter);
            last_track = TrackData(_detect_data.bbox, _detect_data.name, car_index_it, _detect_data.probility);

            max_image_rect = cv::Rect(0, 0, kf_config.width(), kf_config.height());
            overlap_threshold = kf_config.min_overlap_ratio();
            rechek_overlap_ratio = kf_config.recheck_overlap_ratio();
            peron_bicycle_motor.insert("person");
            peron_bicycle_motor.insert("bicycle");
            peron_bicycle_motor.insert("motorcycle");
            peron_bicycle_motor.insert("motorbike");
            car_truck_bus.insert("car");
            car_truck_bus.insert("truck");
            car_truck_bus.insert("bus");
        }

        bool single_tracker::Update(const cv::Mat &src, const std::vector<DetectData> &_detect_data,
                                    const cv::Mat &index_mat, TrackData &track_data,
                                    std::vector<std::pair<float, int> > &index_matched) {
            bool yolo_find = false, kcf_find = false;
            cv::Rect2d tracker_res(0, 0, 0, 0);
            if (kcf_find = (kcf_tracker->update(src, tracker_res)) && tracker_res.area() != 0) {
                tracker_dog->FeedDog();
                last_track.bbox = tracker_res;
            }
            int target_index = -1;
            if (yolo_find = Match(_detect_data, index_mat, last_track, index_matched)) {
                yolo_dog->FeedDog();
                target_index = index_matched.back().second;
            }
            track_data = last_track;

            if (yolo_find && kcf_find) {
                auto bbox = _detect_data[target_index].bbox;
                auto bbox_es = cv::Rect2f(tracker_res);
                bool recheck_flag = false;
                if (track_data.name == "car") {
                    recheck_flag =
                            common::CalculateRectOverlapRatio(bbox, bbox_es, common::AND_MAX) > rechek_overlap_ratio;
                } else {
                    recheck_flag =
                            common::CalculateRectOverlapRatio(bbox, bbox_es, common::AND_OR) > rechek_overlap_ratio;
                }
                //if (track_data.name != "car")
                //    recheck_flag = false;
                if (recheck_flag) {
                    RunKF(_detect_data[target_index].bbox, tracker_res, track_data.bbox);
                    //track_data.bbox = tracker_res;
                    track_data.name = (last_track.name == "person") ? _detect_data[target_index].name : last_track.name;
                } else {
                    track_data.bbox = tracker_res;
                    yolo_find = false;
                }
                track_data.probility = _detect_data[target_index].probility;
            } else if (yolo_find) {
                track_data.bbox = _detect_data[target_index].bbox;
                track_data.probility = _detect_data[target_index].probility;
                track_data.name = (last_track.name == "person") ? _detect_data[target_index].name : last_track.name;
            } else if (kcf_find) {
                track_data.bbox = tracker_res;
                track_data.probility = 101;
            } else {
                track_data.probility = 0;
            }
            /*
            if (kcf_find){
                track_data.probility = 101;
            }else {
                track_data.probility = 0;
            }*/

            //RunMF(video_data.bbox);

            if (yolo_find) {
                kcf_tracker = cv::TrackerKCF::create();
                kcf_tracker->init(src, track_data.bbox);
            }

            last_track = track_data;
            return (tracker_dog->CheckDog() && yolo_dog->CheckDog());
        }

        bool single_tracker::Match(const std::vector<DetectData> &_detect_data, const cv::Mat &index_mat,
                                   const TrackData &last_track, std::vector<std::pair<float, int> > &now_index) {
            /*cv::Rect last_track_bbox(last_track.bbox);
            cv::Rect ROI = last_track_bbox & max_image_rect;
            cv::Mat interest_mat = index_mat(ROI);

            for (int row = 0 ; row < interest_mat.rows; row++)
            {
                int *ptr = interest_mat.ptr<int>(row);
                for (int col = 0 ; col < interest_mat.cols; col++)
                {
                    if (ptr[col] > 0)
                    {
                        float overlap_ratio = common::CalculateRectOverlapRatio(last_track.bbox, _detect_data[ptr[col]].bbox) ;
                        if (overlap_ratio> overlap_threshold && overlap_ratio > max_overlap)
                        {
                            max_overlap = overlap_ratio;
                            max_index = ptr[col];
                        }
                    }
                }
            }*/
            now_index.clear();
            for (int i = 0; i < _detect_data.size(); ++i) {
                float overlap_ratio = common::CalculateRectOverlapRatio(last_track.bbox, _detect_data[i].bbox);

                if (overlap_ratio > overlap_threshold) {
                    if (_detect_data[i].name == last_track.name ||
                        ((car_truck_bus.count(last_track.name)) && car_truck_bus.count(_detect_data[i].name)))
                        now_index.emplace_back(std::pair<float, int>(overlap_ratio, i));
                    else if (peron_bicycle_motor.count(_detect_data[i].name) &&
                             peron_bicycle_motor.count(last_track.name)) {
                        now_index.emplace_back(std::pair<float, int>(overlap_ratio, i));
                        this->last_track.name = "bicycle";
                    }
                }
            }

            if (now_index.empty())return false;
            std::sort(now_index.begin(), now_index.end());
            return (now_index.back().first != 0);
            //if (now_index.back().first == 0)return false;
            // for (auto &item : now_index)std::cout<<item.first<<", ";std::cout<<std::endl;
            //return true;
        }

        void single_tracker::RunKF(const cv::Rect2f &yolo_rect, const cv::Rect2f &kcf_rect, cv::Rect2f &res_rect) {
            VECTOR_TYPE vector_yolo, vector_kcf, vector_res;
            vector_yolo[0] = yolo_rect.tl().x;
            vector_yolo[1] = yolo_rect.tl().y;
            vector_kcf[0] = kcf_rect.tl().x;
            vector_kcf[1] = kcf_rect.tl().y;
            kalman_filter[0].correct(vector_yolo, vector_kcf, vector_res, false);
            cv::Point2f tl(vector_res[0], vector_res[1]);
            vector_yolo[0] = yolo_rect.br().x;
            vector_yolo[1] = yolo_rect.br().y;
            vector_kcf[0] = kcf_rect.br().x;
            vector_kcf[1] = kcf_rect.br().y;
            kalman_filter[1].correct(vector_yolo, vector_kcf, vector_res, false);
            cv::Point2f br(vector_res[0], vector_res[1]);
            res_rect = cv::Rect2f(tl, br);
        }

        void single_tracker::RunMF(cv::Rect2f &new_rect) {
            auto tl = mean_filter[0].Run(new_rect.tl()),
                    br = mean_filter[1].Run(new_rect.br());
            new_rect = cv::Rect2f(tl, br);
        }
    }


}
