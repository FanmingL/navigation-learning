//
// Created by erdou on 18-10-11.
//

#include "BFSeparation.h"
namespace rs{
    namespace vp{

        BFSeparation::BFSeparation()
        {
            SetAlgorithmName("background and foreground algorithm");
            ReadConfig();
            if (bfs_config.if_init()) {
                init_image = cv::Mat(bfs_config.height(), bfs_config.width(), CV_8UC1, cv::Scalar(0));
                stastic_matrix = std::vector<std::vector<std::vector<int> > >(bfs_config.height(),
                                                                              std::vector<std::vector<int> >(
                                                                                      bfs_config.width(),
                                                                                      std::vector<int>(256, 0)));
                ReadData();
            }else{
                for (int i = 1; i <= 4; i++)
                {
                    std::string path = common::get_absolute_path("modules/detect/BFSeparation/config/");
                    path += std::to_string(i);
                    path+=".png";
                    auto img = cv::imread(path, cv::IMREAD_GRAYSCALE);
                    back_ground_vec.push_back(img);
                }
            }
        }

        void BFSeparation::DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res) {
            res.clear();
            static int counter = 0;
            if (bfs_config.if_init()) {
                cv::Mat mask(bfs_config.height(), bfs_config.width(), CV_8UC1, cv::Scalar(255));
                for (auto &item : detect_video.frame(counter).object())
                {
                    cv::Rect2f rect(cv::Rect2f((item.x() - item.width() / 2) * input.cols,
                                               (item.y() - item.height() / 2) * input.rows, item.width() * input.cols,
                                               item.height() * input.rows));
                    rect &= cv::Rect2f(0, 0, input.cols, input.rows);
                    mask(rect) = cv::Scalar(0);
                }
                cv::Mat gray;
                cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
                for (int row = 0; row < input.rows; row++)
                {
                    auto *ptr = gray.ptr<uchar>(row);
                    auto *mask_ptr = mask.ptr<uchar>(row);
                    for (int col = 0 ; col < input.cols; col++)
                    {
                        if (mask_ptr[col] == 0)continue;
                        stastic_matrix[row][col][ptr[col]]++;
                    }
                }
                if (counter && counter % bfs_config.period() == 0)
                {
                    cv::Mat back_ground(cv::Size(input.rows, input.cols), CV_8UC1, cv::Scalar(0));
                    for (int row = 0; row < input.rows; row++)
                    {
                        auto *ptr = back_ground.ptr<uchar>(row);
                        for (int col = 0 ; col < input.cols; col++)
                        {
                            //ptr[col] = (uchar)FindSetMax(stastic_matrix[row][col]);
                            ptr[col] = (uchar)(std::max_element(stastic_matrix[row][col].begin(), stastic_matrix[row][col].end())
                                    - stastic_matrix[row][col].begin());
                            stastic_matrix[row][col] = std::vector<int>(256, 0);
                        }
                    }
                    std::vector<int> imwrite_parameter = {cv::IMWRITE_PNG_COMPRESSION, 0};
                    std::string file_name("modules/detect/BFSeparation/config/");
                    file_name += std::to_string(counter/bfs_config.period());
                    file_name+=".png";
                    cv::imwrite(common::get_absolute_path(file_name), back_ground, imwrite_parameter);
                    init_image = back_ground.clone();
                }
                output = init_image.clone();
            } else{
                cv::Mat gray;
                cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
                int index = counter / bfs_config.period() ;
                index = ((index >= back_ground_vec.size()) ? (int)back_ground_vec.size()-1 : index);
                auto error_positive = gray - back_ground_vec[index];
                auto error_negative = back_ground_vec[index] - gray;
                auto error = (error_positive + error_negative) > bfs_config.binary_threshold();
                auto element = cv::getStructuringElement(CV_SHAPE_ELLIPSE,
                        cv::Size(bfs_config.open_operate_radius(), bfs_config.open_operate_radius()));
                cv::Mat res_mask(bfs_config.height(), bfs_config.width(), CV_8UC1, cv::Scalar(0));
                cv::morphologyEx(error, res_mask, cv::MORPH_OPEN, element);
                auto input_back_up = input.clone();
                output = cv::Scalar(0,0,0);
                input_back_up.copyTo(output,res_mask);
                //input.copyTo(output,res_mask);

               // output =
                //output = input.clone();
            }

            counter++;
        }

        void BFSeparation::ReadConfig() {
            common::ReadProtoFromTextFile("modules/detect/BFSeparation/config/BFSeparation.prototxt", &bfs_config);
        }



        int BFSeparation::FindSetMax(const std::unordered_map<int, int> &in) {
            int max = -1, max_index = -1;
            for (auto &item : in)
            {
                if (item.second > max)
                {
                    max = item.second;
                    max_index = item.first;
                }
            }
            return max_index;
        }

        void BFSeparation::ReadData() {
            common::ReadProtoFromBinaryFile("data/yolo_out_data.proto.b", &detect_video);
        }
    }
}