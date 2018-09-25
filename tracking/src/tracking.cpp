//
// Created by Fanming Luo on 2018/9/25.
//

#include "tracking.h"

tracking::tracking(int _width, int _height)  :
 index_current_frame(0),file_end(false),
basePath(PATH), videoPath(basePath+"/data/RSD-005-stabling.mov"), preconfigPath(basePath+"/data/RSD-005-stabling.txt")
,thresh_overlap(0.5), area_threshold(500.), car_index(0), max_lost_dog(6)
{
    f.open(preconfigPath);
    assert(f.is_open());
    cap.open(videoPath);
    assert(cap.isOpened());
    //imageWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    //imageHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    imageWidth = _width;
    imageHeight = _height;
    imageCount = cap.get(CV_CAP_PROP_FRAME_COUNT);
    resize_ROI = cv::Rect((int)cap.get(CV_CAP_PROP_FRAME_WIDTH) - _width,
            (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) - _height,
            _width, _height);
    /*for (int i = 0; i < 20;i++)
    {
        get_current_frame();
        for (auto &item:obj_current_frame)
            std::cout<<item<<std::endl;
        std::cout<<std::endl;
    }*/
    refresh_pre_file();
}

bool tracking::refresh_pre_file() {
    std::string tmp;
    if (std::getline(f, tmp))
    {
        std::stringstream ss(tmp);
        ss>>data_buf.frame_index>>data_buf.name;
        if (data_buf.name == "cell")
            ss>>data_buf.name;
        ss>>data_buf.bbox.x\
        >>data_buf.bbox.y>>data_buf.bbox.width>>data_buf.bbox.height>>data_buf.probility;
        data_buf.bbox.x -= data_buf.bbox.width/2.0;
        data_buf.bbox.y -= data_buf.bbox.height/2.0;
        data_buf.bbox.x *= imageWidth;
        data_buf.bbox.y *= imageHeight;
        data_buf.bbox.width *= imageWidth;
        data_buf.bbox.height *= imageHeight;
        data_buf.flag_use_to_compare = false;
        return true;
    }
    file_end=true;
    return false;
}

bool tracking::get_current_frame() {
    obj_current_frame.clear();
    {
        std::unique_lock<std::mutex> lock(cap_mutex);
        cap >> current_image;
        cv::Mat image_tmp(current_image, resize_ROI);
        current_image = image_tmp;
    }
    if (current_image.empty()){
        file_end = true;
    }
    if (file_end)return false;
    while ( index_current_frame == data_buf.frame_index)
    {
        obj_current_frame.push_back(data_buf);
        if (!refresh_pre_file())break;
    }
    index_current_frame ++;
    return true;
}

std::ostream & operator << (std::ostream &out,
                            tracking::_data_buf &item) {
    out<<item.frame_index<<", "<<item.name<<", "<<item.bbox;
    return out;
}

void tracking::draw_current_frame(cv::Mat &dst) {
    current_image.copyTo(dst);
    for (auto &item : obj_current_frame)
    {
        cv::Rect rect((int)item.bbox.x, (int)item.bbox.y, (int)item.bbox.width, (int)item.bbox.height);
        cv::rectangle(dst, rect, cv::Scalar(128, 0, 0), 2);
    }
}

void tracking::init_trackers() {
    get_current_frame();
    for (auto &item : obj_current_frame)
    {
        if (item.bbox.area() < area_threshold )continue;
        if (!(item.name == "car" || item.name == "truck" || item.name == "bus" || item.name == "phone"))continue;
        _alm _tmp_alm(car_index++, max_lost_dog, 0.3);
        _tmp_alm.ptr->init(current_image, item.bbox);
        algorithms.push_back(_tmp_alm);
    }
}

bool tracking::run(cv::Mat &dst, std::vector<cv::Rect2d> &res) {
    std::cout<<index_current_frame<<", "<<algorithms.size()<<", "<<car_index<<"\n";
    if (get_current_frame())
    {
        current_image.copyTo(dst);
        res.clear();
        std::vector<int> index_to_delete;
        for (auto iter = algorithms.begin(); iter!= algorithms.end();)
        {
            cv::Rect2d _tmp_rect;
            bool step_res;
            if (!iter->run(current_image, obj_current_frame, _tmp_rect,step_res))
            {
                iter = algorithms.erase(iter);
                continue;
            }else if (step_res)
            {
                res.push_back(_tmp_rect);
                cv::rectangle(dst, _tmp_rect, cv::Scalar(128, 0, 0), 2);
                cv::putText(dst, std::to_string(iter->index), cv::Point(_tmp_rect.x, _tmp_rect.y)
                        ,cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0,0, 128));
            }
            iter++;
        }

        for (auto &item : obj_current_frame)
        {
            if (item.flag_use_to_compare)continue;
            if (item.bbox.area() < area_threshold )continue;
            if (!(item.name == "car" || item.name == "truck" || item.name == "bus" || item.name == "phone"))continue;
            _alm _tmp_alm(car_index++, max_lost_dog, 0.5);
            _tmp_alm.ptr->init(current_image, item.bbox);
            algorithms.push_back(_tmp_alm);
        }
        return true;
    }
    return false;
}


