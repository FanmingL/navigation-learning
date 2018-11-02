//
// Created by erdou on 18-10-15.
//

#include "modules/relocate/draw_area.h"
#include "common/string_util.h"
namespace rs{
    namespace common{
        void CalculateTransform(const cv::Mat &transform_matrix, const cv::Point2f &src_point, cv::Point2f &dst_point, int type=1);
    }
}

namespace rs {
    namespace vp {
        draw_area::draw_area(const cv::Mat &src, const std::string &path, int width, int height, DRAW_USAGE usage,
                const cv::Mat &_homograph_matrix) :
                mask(width, height, CV_8UC1, cv::Scalar(255)), click_step(0) {
            first_image = src.clone();
            if (usage == DRAW_MASK) {
                for (int i = 0; i < 10; i++)
                    val_buffer.push_back(i * 20);
                DrawPosition();
                std::vector<int> parameter = {cv::IMWRITE_PNG_COMPRESSION, 0};
                cv::imwrite(path, mask, parameter);
            }else if (usage == DRAW_POINT){
                homograph_matrix = _homograph_matrix.clone();
                mask = src.clone();
                DrawPoint();
                std::vector<int> parameter = {cv::IMWRITE_PNG_COMPRESSION, 0};
                cv::imwrite(path, mask, parameter);
            }

        }

        void draw_area::MouseCb(int event, int x, int y, int flags) {
            switch (event) {
                case CV_EVENT_LBUTTONDOWN:
                    std::cout << x << ", " << y << std::endl;
                    click_buffer.emplace_back(cv::Point(x, y));
                    break;
                case CV_EVENT_MOUSEMOVE:
                    break;
                case CV_EVENT_LBUTTONUP:
                    break;
                default:
                    break;
            }

        }

        void draw_area::onMouse(int event, int x, int y, int flag, void *user_data) {
            auto *temp = reinterpret_cast<draw_area *>(user_data);
            temp->MouseCb(event, x, y, flag);
        }

        void draw_area::DrawPosition() {
            cv::imshow("first_image", first_image);
            cv::setMouseCallback("first_image", draw_area::onMouse, this);
            int fill_val = 0;
            while (true) {
                canvas = cv::Scalar(0, 0, 0);
                first_image.copyTo(canvas, mask > 230);
                for (auto &item : click_buffer) {
                    cv::circle(canvas, item, 2, cv::Scalar(0, 0, 255), -1);
                }
                for (int i = 1; i < click_buffer.size(); i++) {

                    cv::line(canvas, click_buffer[i - 1], click_buffer[i], cv::Scalar(0, 255, 0), 2);
                }
                cv::putText(canvas, std::to_string(fill_val), cv::Point(10, 10), CV_FONT_BLACK, 0.5,
                            cv::Scalar(0, 0, 255));
                cv::imshow("first_image", canvas);
                auto key = cv::waitKey(30);
                if (key == 'q')break;
                switch (key) {
                    case ' ':
                        cv::fillPoly(mask, std::vector<std::vector<cv::Point> >(1, click_buffer), cv::Scalar(fill_val));
                        click_buffer.clear();
                        break;
                    case 'a':
                        if (!click_buffer.empty())
                            click_buffer.pop_back();
                        break;
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        fill_val = val_buffer[(key - '0')];
                        break;
                    case 'y':
                        fill_val = 200;
                        break;
                    default:
                        break;
                }
            }
            std::vector<int> parameters;
            parameters.push_back(CV_IMWRITE_PNG_COMPRESSION);
            parameters.push_back(0);
            //cv::imwrite(base_path + "/data/cant_touch_place_mask.png", mask, parameters);
        }



        void draw_area::ReadPosition() {
            mask_cant_reach = cv::imread(base_path + "/data/cant_touch_place_mask.png", cv::IMREAD_GRAYSCALE);

            cv::imshow("111", mask_cant_reach);
            cv::waitKey();
        }

        void draw_area::DrawPoint() {
            cv::imshow("draw_point", mask);
            cv::setMouseCallback("draw_point", draw_area::onMouse, this);
            while (true){

                canvas = mask.clone();
                for (auto & item : click_buffer){
                    cv::circle(canvas,item,2,cv::Scalar(0,0,255),-1);
                    cv::Point2f dst;
                    common::CalculateTransform(homograph_matrix,item,dst);
                    cv::putText(canvas,common::to_string_with_precision(dst.x,4),item,cv::FONT_ITALIC,0.5,cv::Scalar(128,0,0),2);
                    cv::putText(canvas,common::to_string_with_precision(dst.y,4),item + cv::Point(0, 15),cv::FONT_ITALIC,0.5,cv::Scalar(128,0,0),2);
                }
                cv::imshow("draw_point", canvas);


                auto key = cv::waitKey(30);
                if (key == 'q')break;
                switch (key){
                    case 'a':
                        if (!click_buffer.empty()){
                            click_buffer.pop_back();
                        }
                        break;
                    default:
                        break;
                }
            }
            mask = canvas.clone();
        }
    }
}