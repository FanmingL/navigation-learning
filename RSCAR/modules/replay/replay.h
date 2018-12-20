/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : replay.h
*   Author      : FanmingL
*   Created date: 2018-12-17 12:40:33
*   Description : 
*
*===============================================================*/


#ifndef _REPLAY_H
#define _REPLAY_H

#include "common/main_interface.h"
#include "common/io.h"
#include "common/rs.h"
#include "common/image_util.h"

#include "modules/replay/replay.pb.h"
#include "modules/information/information.pb.h"
#include "modules/relocate/relocate.pb.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/dnn.hpp"

#include <fstream>

#include <torch/script.h>
#include <torch/csrc/autograd/variable.h>
#include <torch/csrc/autograd/function.h>


namespace rs {
    namespace vp {
        class FrameData {
            friend std::ostream & operator<<(std::ostream &out, FrameData &obj);
        public:
            FrameData(const int &id_, const float &x_, const float &y_, const float &v_x_, const float &v_y_,
                      const float &target_x_ = 0, const float &target_y_ = 0, const float &vx_next_ = 0,
                      const float &vy_next_ = 0
            ) :
                    id(id_), x(x_), y(y_), v_x(v_x_), v_y(v_y_), vx_next(vx_next_), vy_next(vy_next_),
                    target_x(target_x_), target_y(target_y_) {}

            int id;
            float x;
            float y;
            float v_x;
            float v_y;
            float vx_next;
            float vy_next;
            float target_x;
            float target_y;
        };


        class Frame {
            friend std::ostream & operator<<(std::ostream &out, Frame &obj);
        public:
            std::vector<FrameData> data;
        };

        class AllFrame {
        public:
            AllFrame(int max_count = 15000) : data(15000) {}

            std::vector<Frame> data;
        };

        class replay : public rs::common::rs {
        public:
            explicit replay(const std::string &name);

            ~replay() override = default;

            void Run() override;

            void ReadConfig();

            void ReadData();

            void GetHomography(cv::Mat &homography_matrix);

            void GetFeatureWithLabel(cv::Mat &canvas, const cv::Mat &velocity_x,
                                     const cv::Mat &velocity_y, const FrameData &data,
                                     std::vector<float> &feature_with_label);

            void GetFeatureOnly(const cv::Mat &canvas, const cv::Mat &velocity_x,
                                const cv::Mat &velocity_y, const FrameData &data, std::vector<float> &feature);

            void InitBackground(const cv::Mat &original, const cv::Mat &H, cv::Mat &out);

            void DrawCanvas(cv::Mat &canvas_, cv::Mat &vx_mask, cv::Mat &vy_mask, const Frame &frame);

            bool ShowAndSave(const cv::Mat &canvas, int time = 3);

            void ExtractFeature(std::vector<float> &features, const cv::Mat &canvas_,
                                const cv::Mat &velocity_x, const cv::Mat &velocity_y, const cv::Point &pit,
                                const float &cfa, const float &sfa, const float &abs_velocity, const cv::Rect &rect_
            );

            void SaveOrNew(std::ofstream &of);

            void InitFrame(Frame &data);

            void SetNextFrame(Frame &inout_data, const cv::Mat &canvas, const cv::Mat &velocity_x,
                              const cv::Mat &velocity_y);

            void CalculateOut(const std::vector<float> &feature, std::vector<float> &out);

            void SetNextVelocity(const std::vector<float> &out, FrameData &car);

        private:

            ReplayConfig config;

            AllTrajectory all_trajectories;

            AllFrame all_frame;

            PointPairSet point_pair;

            cv::Mat background_mask, bg_image;

            cv::VideoWriter video_writer;

            const int HEIGHT_BASE = 80, WIDTH_BASE = 50, CAR_WIDTH = 2, CAR_HEIGHT = 3;
            const int CAR_INT = 1, NOTHING_INT = 0, OBSTACLE_INT = -1;

            std::ofstream of;

            int xlim, ylim;

            float xinterval, yinterval, ratio;

            cv::Rect max_rect;
            cv::Mat show;
            int frame_count;

            DataArray data_array;
            std::vector<float> w_angle, w_velocity;
            cv::Mat ca_show;

            cv::dnn::Net pre_trained_net;

            std::shared_ptr<torch::jit::script::Module> torch_model;
        };
    }
};

#endif //REPLAY_H
