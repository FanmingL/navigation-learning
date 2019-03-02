/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : replay.cpp
*   Author      : FanmingL
*   Created date: 2018-12-17 12:40:29
*   Description : 
*
*===============================================================*/


#include "replay.h"

namespace rs {
    namespace vp {

        replay::replay(const std::string &name) : rs(name) {
            ReadConfig();
            ReadData();
            ratio = config.ratio();
            xlim = config.xlim();
            ylim = config.ylim();

            xinterval = config.xinterval();
            yinterval = config.yinterval();
            frame_count = 0;
        }

        void replay::Run() {
            cv::Mat H;
            GetHomography(H);
            std::cout<<H<<std::endl;
            int feature_length = 0;
            cv::Mat canvas((int) (HEIGHT_BASE * ratio), (int) (WIDTH_BASE * ratio), CV_8UC1, cv::Scalar(0));
            ca_show = canvas.clone();
            max_rect = cv::Rect(cv::Point(0, 0), cv::Point(canvas.cols - 1, canvas.rows - 1));
            if (config.if_write_video()) {
                std::remove((char *) common::GetAbsolutePath("data/scan_replay.mp4").c_str());
                video_writer.open(common::GetAbsolutePath("data/scan_replay.mp4"), CV_FOURCC('D', 'I', 'V', 'X'), 30,
                                  cv::Size(WIDTH_BASE, HEIGHT_BASE) * (int) (ratio * config.double_ratio()), false
                );
            }

            background_mask = canvas.clone();
            InitBackground(bg_image, H, background_mask);
            cv::Mat velocity_x_mask((int) (HEIGHT_BASE * ratio), (int) (WIDTH_BASE * ratio), CV_32FC1,
                                    cv::Scalar(0, 0)), velocity_y_mask((int) (HEIGHT_BASE * ratio),
                                                                       (int) (WIDTH_BASE * ratio), CV_32FC1,
                                                                       cv::Scalar(0, 0));
            if (config.if_test()){
                Frame frame;
                InitFrame(frame);
                while (true){
                    canvas = background_mask.clone();
                    velocity_x_mask = cv::Scalar(0);
                    velocity_y_mask = cv::Scalar(0);
                    frame_count++;
                    DrawCanvas(canvas, velocity_x_mask, velocity_y_mask, frame);
                    //std::cout<<frame;
                    canvas.copyTo(ca_show);
                    SetNextFrame(frame, canvas, velocity_x_mask, velocity_y_mask);
                    for (auto &item : frame.data){
                        cv::putText(ca_show,std::to_string(item.id),
                                cv::Point2f(item.x, item.y) * ratio, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(100));
                    }
                    if (ShowAndSave(ca_show, 0 ))break;
                }
            }else {
                for (const auto &frame : all_frame.data) {
                    canvas = background_mask.clone();
                    velocity_x_mask = cv::Scalar(0);
                    velocity_y_mask = cv::Scalar(0);
                    frame_count++;
                    DrawCanvas(canvas, velocity_x_mask, velocity_y_mask, frame);
                    SaveOrNew(of);
                    for (const auto &car : frame.data) {
                        std::vector<float> features;
                        GetFeatureWithLabel(canvas, velocity_x_mask, velocity_y_mask, car, features);
                        if (config.if_write_data()) {
                            for (auto &item : features)of << item << ", ";
                            of << std::endl;
                        }
                        if (feature_length==0)feature_length = (int)features.size();
                    }
                    if (ShowAndSave(canvas))break;
                    std::cout << frame_count << std::endl;
                }
            }
            std::cout<<"feature length: "<<feature_length<<std::endl;
        }

        void replay::ReadConfig() {
            common::ReadProtoFromTextFile("modules/replay/config/replay.prototxt", &config);
        }

        void replay::ReadData() {
            all_frame.data.resize((size_t) config.max_frame_count());
#if 0
            for (int i = 0 ; i <= config.max_count(); ++i){
                DescriptionTrajectory trajectory;
                common::ReadProtoFromBinaryFile(config.data_path() + std::to_string(i-1) + ".proto.bin", &trajectory);
                auto &target_item =  trajectory.trajectory_point(trajectory.trajectory_point_size()-1);
                for (int j = 0; j < trajectory.trajectory_point_size()-1; ++j){
                    auto &item = trajectory.trajectory_point(j);
                    auto &next_item = trajectory.trajectory_point(j+1);
                    if (j > 15)
                    all_frame.data[item.frame_index()].data.emplace_back(item.object_index(),item.world_position_x(),\
                    item.world_position_y(), item.world_velocity_x(), item.world_velocity_y(),
                    target_item.world_position_x(), target_item.world_position_y(),
                    next_item.world_velocity_x(), next_item.world_velocity_y()
                    );
                }
            }
            AllFrameProto frame_proto;
            for (int i = 0 ; i < all_frame.data.size(); ++i){
                auto &frame = all_frame.data[i];
                auto *iter = frame_proto.add_data();
                iter->set_frame_id(i+1);
                for (const auto &car : frame.data){
                    auto *iter2 = iter->add_data();
                    iter2->set_id(car.id);
                    iter2->set_x(car.x);
                    iter2->set_y(car.y);
                    iter2->set_vx(car.v_x);
                    iter2->set_vy(car.v_y);
                    iter2->set_target_x(car.target_x);
                    iter2->set_target_y(car.target_y);
                    iter2->set_vx_next(car.vx_next);
                    iter2->set_vy_next(car.vy_next);
                }
            }
            common::WriteProtoToBinaryFile("data/replay_data.protobin", &frame_proto);
#else

            AllFrameProto frame_proto;
            common::ReadProtoFromBinaryFile("data/replay_data.protobin", &frame_proto);
            for (auto &item : frame_proto.data()) {
                int count = 0;
                for (auto &item1 : item.data()) {
                    all_frame.data[item.frame_id()].data.emplace_back(item1.id(), item1.x() / 100 + 18, \
                    item1.y() / 100, item1.vx(), item1.vy(),
                    item1.target_x() / 100 + 18, item1.target_y() / 100, item1.vx_next(), item1.vy_next());
                }
            }
#endif
            common::ReadProtoFromTextFile("modules/relocate/config/PointsSetCar.prototxt", &point_pair);
            bg_image = cv::imread(common::GetAbsolutePath("data/mask-car.png"), cv::IMREAD_GRAYSCALE);
            common::ReadProtoFromTextFile("modules/replay/config/w_angle.prototxt", &data_array);
            for (auto &item : data_array.data())w_angle.push_back(item);
            common::ReadProtoFromTextFile("modules/replay/config/w_velocity.prototxt", &data_array);
            for (auto &item : data_array.data())w_velocity.push_back(item);
            //pre_trained_net = cv::dnn::readNetFromTorch(common::GetAbsolutePath("data/model.pt"));
            torch_model=  torch::jit::load(common::GetAbsolutePath("data/model.pt"));
            //  pre_trained_net.setInput()


            //std::cout<<w_angle.size()<<", "<<w_velocity.size()<<std::endl;


        }

        void replay::GetHomography(cv::Mat &_homograph_matrix) {
            std::vector<cv::Point2f> image_points, world_points;
            for (auto &item : point_pair.points()) {
                image_points.emplace_back(cv::Point2f(item.x_pixel(), item.y_pixel()));
                world_points.emplace_back(cv::Point2f(item.x_world(), item.y_world()));
            }
            cv::Mat tmp_val = cv::findHomography(image_points, world_points);
            _homograph_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    _homograph_matrix.at<float>(i, j) = (float) tmp_val.at<double>(i, j);
                }
        }

        void replay::GetFeatureWithLabel(cv::Mat &canvas, const cv::Mat &velocity_x, const cv::Mat &velocity_y,
                                         const FrameData &car, std::vector<float> &features) {
            features.clear();
            float face_angle = atan2f(car.v_y, car.v_x);
            float target_angle = atan2f(car.target_y - car.y, car.target_x - car.x);
            float next_angle = atan2f(car.vy_next, car.vx_next);
            float cfa = cosf(face_angle), sfa = sinf(face_angle);
            float abs_velocity = sqrtf(car.v_x * car.v_x + car.v_y * car.v_y);
            float abs_velocity_next = sqrtf(car.vx_next * car.vx_next + car.vy_next * car.vy_next);
            float next_prij_vy = cosf(next_angle - face_angle) * abs_velocity_next,
            next_prij_vx = sinf(next_angle - face_angle) * abs_velocity_next;
            features.push_back(next_angle - face_angle);
            /*if (abs_velocity != 0)
                features.push_back((abs_velocity_next - abs_velocity) / abs_velocity);
            else
                features.push_back(0);
            */
            features.push_back((abs_velocity_next - abs_velocity));
            //features.push_back(next_prij_vx);
            //features.push_back(next_prij_vy - abs_velocity);
            features.push_back((target_angle - face_angle));
            features.push_back(cosf(target_angle - face_angle));
            features.push_back(sinf(target_angle - face_angle));
            cv::Point2f p_center(car.x, car.y), yc_t(cfa, sfa),
                    xc_t(sfa, -cfa);
            cv::Rect rect((int) ((car.x - CAR_WIDTH / 2.0f) * ratio),
                          (int) ((car.y - CAR_HEIGHT / 2.0f) * ratio), \
                            (int) (ratio * CAR_WIDTH), (int) (ratio * CAR_HEIGHT));
            for (int y_ = -ylim; y_ < ylim; ++y_) {
                for (int x_ = -xlim; x_ < xlim; ++x_) {
                    cv::Point p_i_t = (p_center + y_ * yinterval * yc_t + x_ * xinterval * xc_t) * ratio;
                    if (config.if_draw_area() && max_rect.contains(p_i_t))
                         canvas.at<uchar>(p_i_t) = 160;
                    ExtractFeature(features, canvas, velocity_x, velocity_y, p_i_t, cfa, sfa, abs_velocity, rect);
                }
            }
        }

        void replay::GetFeatureOnly(const cv::Mat &canvas, const cv::Mat &velocity_x, const cv::Mat &velocity_y,
                                    const FrameData &car, std::vector<float> &features) {
            features.clear();
            float face_angle = atan2f(car.v_y, car.v_x);
            float target_angle = atan2f(car.target_y - car.y, car.target_x - car.x);
            float cfa = cosf(face_angle), sfa = sinf(face_angle);
            float abs_velocity = sqrtf(car.v_x * car.v_x + car.v_y * car.v_y);
            features.push_back(target_angle - face_angle);
            features.push_back(cosf(target_angle - face_angle));
            features.push_back(sinf(target_angle - face_angle));
            cv::Point2f p_center(car.x, car.y), yc_t(cfa, sfa),
                    xc_t(sfa, -cfa);
            cv::Rect rect((int) ((car.x - CAR_WIDTH / 2.0f) * ratio),
                          (int) ((car.y - CAR_HEIGHT / 2.0f) * ratio), \
                            (int) (ratio * CAR_WIDTH), (int) (ratio * CAR_HEIGHT));
            for (int y_ = -ylim; y_ < ylim; ++y_) {
                for (int x_ = -xlim; x_ < xlim; ++x_) {
                    cv::Point p_i_t = (p_center + y_ * yinterval * yc_t + x_ * xinterval * xc_t) * ratio;
                    if (config.if_draw_area() && max_rect.contains(p_i_t))
                        ca_show.at<uchar>(p_i_t) = 170;
                    ExtractFeature(features, canvas, velocity_x, velocity_y, p_i_t, cfa, sfa, abs_velocity, rect);
                }
            }
        }

        void replay::InitBackground(const cv::Mat &original, const cv::Mat &H, cv::Mat &out) {
            for (int i = 0; i < original.rows; ++i) {
                const uchar *iter = original.ptr(i);
                for (int j = 0; j < original.cols; ++j) {
                    if (*(iter + j) < 10) {
                        cv::Point2f res;
                        common::CalculateTransform(H, cv::Point2f(j, i), res);
                        res = (res / 100 + cv::Point2f(18, 0)) * ratio;
                        cv::Point res1 = res;
                        if (max_rect.contains(res))
                            out.at<uchar>(res1) = 128;
                    }
                }
            }
        }

        void
        replay::DrawCanvas(cv::Mat &canvas_, cv::Mat &velocity_x_mask_, cv::Mat &velocity_y_mask_, const Frame &frame) {
            for (const auto &car : frame.data) {
                cv::Rect rect((int) ((car.x - CAR_WIDTH / 2.0f) * ratio),
                              (int) ((car.y - CAR_HEIGHT / 2.0f) * ratio), \
                    (int) (ratio * CAR_WIDTH), (int) (ratio * CAR_HEIGHT));
                canvas_(rect & max_rect) = cv::Scalar(255);
                velocity_x_mask_(rect & max_rect) = cv::Scalar(car.v_x);
                velocity_y_mask_(rect & max_rect) = cv::Scalar(car.v_y);
                if (config.if_draw_target()) {
                    cv::Point2f target_p(car.target_x, car.target_y);
                    target_p = target_p * ratio;
                    cv::line(canvas_, (cv::Point2f(rect.tl()) + cv::Point2f(rect.br())) / 2, target_p,
                             cv::Scalar(200), 1, cv::LINE_AA);
                }
            }
        }

        bool replay::ShowAndSave(const cv::Mat &canvas, int time) {
            if (config.if_show_video() || config.if_write_video()) {
                cv::resize(canvas, show, cv::Size(WIDTH_BASE, HEIGHT_BASE) * (int) (ratio * config.double_ratio())\
, 0, 0, cv::INTER_NEAREST);
                if (config.if_show_video()) {
                    cv::imshow("1", show);
                    if (cv::waitKey(time) == 'q')return true;
                }
                if (config.if_write_video()) {
                    if (frame_count % 1 == 0) {
                        video_writer << show;
                    }
                }
            }
            return false;
        }

        void replay::ExtractFeature(std::vector<float> &features, const cv::Mat &canvas_, const cv::Mat &velocity_x,
                                    const cv::Mat &velocity_y, const cv::Point &p_i_t, const float &cfa,
                                    const float &sfa, const float &abs_velocity, const cv::Rect &rect_
        ) {
#define IF_WRITE_TYPE 1
            if (rect_.contains(p_i_t)) {
#if IF_WRITE_TYPE
                features.push_back(1);
                features.push_back(0);
                features.push_back(0);
                /// NOTHING
#endif
                features.push_back(0);
                features.push_back(0);
            } else if (max_rect.contains(p_i_t)) {
                switch (canvas_.at<uchar>(p_i_t)) {
                    case 255: {
#if IF_WRITE_TYPE
                        features.push_back(0);
                        features.push_back(1);
                        features.push_back(0);
                        //features.push_back(CAR_INT);
#endif
                        float vx_ttt = velocity_x.at<float>(p_i_t),
                                vy_ttt = velocity_y.at<float>(p_i_t);
                        features.push_back((vx_ttt * sfa - vy_ttt * cfa));
                        features.push_back((vx_ttt * cfa + vy_ttt * sfa - abs_velocity));
                        break;
                    }
                    case 128: {
#if IF_WRITE_TYPE
                        features.push_back(0);
                        features.push_back(0);
                        features.push_back(1);
                        //features.push_back(OBSTACLE_INT);
#endif
                        features.push_back(0);
                        features.push_back(-abs_velocity);
                        //features.pu
                        break;
                    }
                    case 0: {
#if IF_WRITE_TYPE
                        features.push_back(1);
                        features.push_back(0);
                        features.push_back(0);
                        //features.push_back(NOTHING_INT);
#endif
                        features.push_back(0);
                        features.push_back(0);
                        break;
                    }
                    default: {
#if IF_WRITE_TYPE
                        features.push_back(1);
                        features.push_back(0);
                        features.push_back(0);
                        //features.push_back(NOTHING_INT);
#endif
                        features.push_back(0);
                        features.push_back(0);
                        break;
                    }
                }
            } else {
#if IF_WRITE_TYPE
                features.push_back(1);
                features.push_back(0);
                features.push_back(0);
                //features.push_back(NOTHING_INT);
#endif
                features.push_back(0);
                features.push_back(0);
            }
        }

        void replay::SaveOrNew(std::ofstream &of_) {
            static int file_count = -1;
            if (config.if_write_data() && file_count != frame_count /config.frame_per_file()) {
                if (file_count >= 0)of_.close();
                file_count = frame_count / config.frame_per_file();
                of_.open(common::GetAbsolutePath("data/replay_dir/replay") + std::to_string(file_count) + ".txt",
                        std::ios::trunc);
            }
        }

        void replay::InitFrame(Frame &data) {
            /*                     id, x, y, vx, vy, target_x, target_y */
            FrameProto frame_data_proto;
            common::ReadProtoFromTextFile("modules/replay/config/init_frame.prototxt", &frame_data_proto);
            for (auto &item : frame_data_proto.data()){
                data.data.emplace_back(item.id(), item.x(),
                        item.y(), item.vx(), item.vy(), item.target_x(), item.target_y());
            }


        }

        void replay::SetNextFrame(Frame &inout_data, const cv::Mat &canvas, const cv::Mat &velocity_x,
                                  const cv::Mat &velocity_y) {
            Frame frame_out;
            for (auto &car : inout_data.data){
                std::vector<float> feature, out;
                GetFeatureOnly(canvas, velocity_x, velocity_y, car,feature);
                CalculateOut(feature, out);
                std::cout<<car.id<<", angle add: "<<out[0]<<", velocity_add:  "<<out[1]<<std::endl;
                car.x += car.v_x;
                car.y += car.v_y;
                SetNextVelocity(out, car);
                if (max_rect.contains(cv::Point2f(car.x, car.y) * ratio))
                {
                    frame_out.data.push_back(car);
                }
            }
            //std::
            inout_data.data.clear();
            inout_data.data = frame_out.data;
        }

        void replay::CalculateOut(const std::vector<float> &feature, std::vector<float>& out) {
            out.clear();
            out.resize(2);
#if 0
            out[0] = out[1] = 0;
            for (int i = 0; i < feature.size(); ++i){
                out[0] += feature[i] * w_angle[i];
                out[1] += feature[i] * w_velocity[i];
            }
#else
            std::vector<torch::jit::IValue> inputs;
            torch::Tensor in = torch::zeros({(int)feature.size()}, torch::requires_grad(false));
            float *iter=  in.data<float>();
            for (int i = 0; i < feature.size(); ++i){
                *(iter + i) = feature[i];
            }
            inputs.emplace_back(in);
            auto res = torch_model->forward(inputs).toTensor();
            iter = res.data<float>();
            out[0] = *iter;
            out[1] = *(iter+1);
#endif
        }

        void replay::SetNextVelocity(const std::vector<float> &out, FrameData &car) {
                float abs_velocity = sqrtf(car.v_x * car.v_x + car.v_y * car.v_y),
                angle = atan2f(car.v_y, car.v_x);
                abs_velocity += out[1];
                angle += out[0];
               /*
                abs_velocity += out[1] * config.velocity_calc_ratio();
                car.v_x = abs_velocity * cosf(angle) - out[0] * sinf(angle) * config.angle_calc_ratio();
                car.v_y = abs_velocity * sinf(angle) + out[0] * cosf(angle) * config.angle_calc_ratio();
                */
               car.v_x = abs_velocity * cosf(angle);
               car.v_y = abs_velocity * sinf(angle);
        }

        std::ostream &operator<<(std::ostream &out, FrameData &obj) {
            out << "id: "<<obj.id<<", x: "<<obj.x<<", y: "<<obj.y
            <<", vx: "<<obj.v_x<<", vy: "<<obj.v_y<<", angle: "<<(atan2(obj.v_y, obj.v_x)/((float)CV_PI)*180)
            <<", target angle: "<<(atan2(obj.target_y - obj.y, obj.target_x - obj.x))/((float)CV_PI)*180<<std::endl;
            return out;
        }

        std::ostream &operator<<(std::ostream &out, Frame &obj) {
            for (auto &item : obj.data){
                out<<item;
            }
            out<<std::endl;
            return out;
        }
    }
}

MAIN(rs::vp::replay, "replay");
