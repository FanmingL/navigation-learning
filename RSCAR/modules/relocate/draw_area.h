//
// Created by erdou on 18-10-15.
//

#ifndef RSCAR_DRAW_AREA_H
#define RSCAR_DRAW_AREA_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <mutex>

namespace rs {
    namespace vp {
        class draw_area {
        public:
            enum DRAW_USAGE{
                DRAW_MASK = 0,
                DRAW_POINT = 1
            };
            draw_area(const cv::Mat &src, const std::string &path, int width = 1080, int height = 1080
                    , DRAW_USAGE usage = DRAW_MASK, const cv::Mat &homograph_matrix = cv::Mat());

            cv::Mat first_image, canvas, mask, mask_cant_reach;

            void DrawPosition();

            void ReadPosition();

            const std::string base_path = PATH;

            void MouseCb(int event, int x, int y, int flags);

            static void onMouse(int event, int x, int y, int flag, void *user_data);

            void DrawPoint();

        private:
            std::mutex init_image_mutex;

            int click_step;

            std::vector<cv::Point> click_buffer;

            std::vector<int> val_buffer;

            cv::Mat homograph_matrix;
        };
    }
}

#endif //RSCAR_DRAW_AREA_H
