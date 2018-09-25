//
// Created by Fanming Luo on 2018/9/25.
//

#ifndef TRACKING_TRACKING_H
#define TRACKING_TRACKING_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/tracking.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <algorithm>
#include <mutex>
class tracking {
public:
    explicit tracking(int width=960, int height=960);
    ~tracking()= default;
    bool get_current_frame();
    void draw_current_frame(cv::Mat &dst);
    void init_trackers();
    bool run(cv::Mat &dst, std::vector<cv::Rect2d> &res);
    std::string basePath, videoPath, preconfigPath, resPath, nameAlg;
    class _data_buf{
        friend std::ostream & operator<<(std::ostream &out, _data_buf &obj);
    public:
        _data_buf():flag_use_to_compare(false), frame_index(0), probility(0.0f){}
        int frame_index;
        std::string name;
        cv::Rect2d bbox;
        float probility;
        bool flag_use_to_compare;
    }data_buf;

    class _alm{
    public:
        _alm(int _index, int _max_count=10, double _threshold=0.5):ptr(cv::TrackerKCF::create()), count(0), max_count(_max_count),
        threshold(_threshold), index(_index)
        {}
        ~_alm() = default;
        void feed_dog(){count = 0;}
        double overlap_judge(cv::Rect2d &rect1, cv::Rect2d &rect2) {
            double a1 = rect1.area(),a2=rect2.area();
            if (a1 == 0 || a2==0)return -1;
            return (rect1 & rect2).area() / std::min(a1, a2);
        }
        bool check_dog(){if (++count>=max_count)return false;return true;}
        bool run(cv::Mat &m, std::vector<tracking::_data_buf> &yolo_res, cv::Rect2d &res, bool &step_res){
            cv::Rect2d r;
            step_res=ptr->update(m, r);
            res = r;
            for (auto &item : yolo_res)
            {
                if (item.flag_use_to_compare)continue;
                if (item.bbox.area() < 500 )continue;
                if (!(item.name == "car" || item.name == "truck" || item.name == "bus" || item.name == "phone"))continue;
                if (overlap_judge(r, item.bbox) > threshold)
                {
                    ptr->init(m, item.bbox);
                    feed_dog();
                    res = item.bbox;
                    item.flag_use_to_compare = true;
                    // break;
                }
            }
            return check_dog();
        }

        cv::Ptr<cv::Tracker> ptr;
        int count, max_count, index;
        double threshold;
    };
    cv::Mat current_image;
    cv::MultiTracker trackers;
    int car_index;
private:
    bool refresh_pre_file();
    const double thresh_overlap, area_threshold;
    cv::VideoCapture cap;
    const int max_lost_dog;

    double imageWidth, imageHeight, imageCount;
    std::list<_alm>  algorithms;
    std::vector<cv::Rect2d> objects, ROIs;
    cv::Rect resize_ROI;
    std::ifstream f;
    std::mutex cap_mutex;
    std::vector<_data_buf> obj_current_frame;
    int index_current_frame;
    bool file_end;
};


#endif //TRACKING_TRACKING_H
