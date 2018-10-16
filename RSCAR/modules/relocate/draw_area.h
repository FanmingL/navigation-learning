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
            draw_area(const cv::Mat &src, const std::string &path, int width = 1080, int height = 1080);

            cv::Mat first_image, canvas, mask, mask_cant_reach;

            void draw_position();

            void read_position();

            const std::string base_path = PATH;

            void mouse_cb(int event, int x, int y, int flags);

            static void onMouse(int event, int x, int y, int flag, void *user_data);

        private:
            std::mutex init_image_mutex;

            int click_step;

            std::vector<cv::Point> click_buffer;

            std::vector<int> val_buffer;
        };
    }
}

#endif //RSCAR_DRAW_AREA_H
