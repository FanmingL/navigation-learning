//
// Created by erdou on 18-10-13.
//

#ifndef _RSCAR_KFILTER_H
#define _RSCAR_KFILTER_H

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "modules/track/base_track_algorithm.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include "modules/track/KFilter/KalmanFilter.h"
#include "modules/detect/detect_algorithm_base.h"
#include "modules/track/KFilter/KFilter.pb.h"
#include <vector>
#include "common/mean_filter.h"
#include "opencv2/tracking.hpp"
#include "common/util.h"
#include <list>
#include "common/image_util.h"
namespace rs{
    namespace vp{

        class single_tracker{
        public:
            single_tracker(const cv::Mat &src, const DetectData &_detect_data, const KFilterConfig &kf_config);

            bool Update(const cv::Mat &src, const std::vector<DetectData> & _detect_data,
                    const cv::Mat &index_mat, TrackData &track_data, std::vector<std::pair<float, int> > &index_matched);

            bool Match(const std::vector<DetectData> &_detect_data, const cv::Mat &index_mat,
                    const TrackData &last_track, std::vector<std::pair<float, int> > &now_index);

            void RunKF(const cv::Rect2f &yolo_rect, const cv::Rect2f &kcf_rect, cv::Rect2f &res_rect);

            void RunMF(cv::Rect2f &new_rect);
            std::vector<common::mean_filter<cv::Point2f> > mean_filter;
            std::vector<KalmanFilter> kalman_filter;
            DetectData detect_data;
            cv::Ptr<cv::TrackerKCF> kcf_tracker;
            std::shared_ptr<common::WatchDog> yolo_dog, tracker_dog;
            int car_index_it;
            TrackData  last_track;
            cv::Rect max_image_rect;
            float overlap_threshold;
            float rechek_overlap_ratio;
            static int car_index_counter;
            std::unordered_set<std::string> peron_bicycle_motor;
        };

        class KFilter : public BaseTrackAlgorithm{
        public:
            KFilter();
            ~KFilter() override = default;
            void Track(const cv::Mat &src, cv::Mat &dst,
                               const DetectFrame &detect_frame, std::vector<TrackData> &res) override;
            void ReadConfig();
        private:
            KFilterConfig kf_config;
            bool CheckConstraint(const DetectData &object);
            cv::Mat ConvertData(const DetectFrame &detect_frame, std::vector<DetectData> &detect_frame_data );
            std::list<single_tracker> single_tracker_list;
            void init_names();
            cv::Rect2f GetRect(const DetectObject &object);
            std::unordered_set<std::string> names_should_take_care;
            std::unordered_map<std::string, cv::Scalar> color_map;
            cv::Rect2f max_image_rect;
            cv::Mat road_type_mask;
            std::unordered_set<std::string> peron_bicycle_motor;
        };

        rs::common::REGISTER_ALGORITHM(BaseTrackAlgorithm, "KFilter", KFilter);

    }
}


#endif //RSCAR_KFILTER_H
