//
// Created by erdou on 18-10-11.
//

#include "YoloDetection.h"

namespace rs{
    namespace vp{

        void YoloDetection::ReadConfig() {
            common::ReadProtoFromTextFile("modules/detect/Yolo/config/YoloConfig.prototxt", &yolo_config);
        }

        void YoloDetection::DetectObject(const cv::Mat &input, cv::Mat &output, std::vector<DetectData> &res) {
            SetAlgorithmName("You Only Look Ones!!!");
            int nboxes = 0;
            image _tmp_image = mat_to_image(input);
            image _sized = letterbox_image(_tmp_image, net->w, net->h);

            layer *l = &(net->layers[net->n - 1]);
            network_predict(net, _sized.data);
            detection *dets = get_network_boxes(net, _tmp_image.w, _tmp_image.h, yolo_config.threshold(), yolo_config.hier(), nullptr, 1, &nboxes);
            if (yolo_config.if_nms())do_nms_sort(dets, nboxes, l->classes, yolo_config.nms());
            draw_detections_new(_tmp_image, dets, nboxes, yolo_config.threshold(), l->classes, res);
            output = image_to_mat(_tmp_image);
            free_detections(dets, nboxes);
            free_image(_sized);
            free_image(_tmp_image);
        }

        YoloDetection::YoloDetection() {
            ReadConfig();
            names = get_labels((char *) common::get_absolute_path(yolo_config.name_config_path()).c_str());
            net = load_network((char *) common::get_absolute_path(yolo_config.net_config_path()).c_str(),
                    (char *) common::get_absolute_path(yolo_config.net_weight_path()).c_str(), 0);
            set_batch_network(net, 1);
            if (!(yolo_config.net_input_width() == 608 && yolo_config.net_input_height() == 608))
            {
                resize_network(net, yolo_config.net_input_width(), yolo_config.net_input_height());
            }
            load_alphabet_new();
        }

        void YoloDetection::load_alphabet_new() {
            int i, j;
            const int nsize = 8;
            alphabets = (image **) calloc(nsize, sizeof(image));
            for (j = 0; j < nsize; ++j) {
                alphabets[j] = (image *) calloc(128, sizeof(image));
                for (i = 32; i < 127; ++i) {
                    char buff[256];
                    sprintf(buff, "%s/%d_%d.png", (char *) common::get_absolute_path(yolo_config.label_path()).c_str(), i, j);
                    alphabets[j][i] = load_image_color(buff, 0, 0);
                }
            }
        }

        void YoloDetection::draw_detections_new(image im, detection *dets, int num, float thresh, int classes,
                                                std::vector<DetectData> &res_name) {
            int i, j;
            res_name.clear();
            for (i = 0; i < num; ++i) {
                char labelstr[4096] = {0};
                int class1 = -1;
                for (j = 0; j < classes; ++j) {
                    if (dets[i].prob[j] > thresh) {
                        if (class1 < 0) {
                            strcat(labelstr, names[j]);
                            class1 = j;
                        } else {
                            strcat(labelstr, ", ");
                            strcat(labelstr, names[j]);
                        }
                        //printf("%s: %.0f%%\n", names[j], dets[i].prob[j]*100);
                        //YOLO_OUT y(names[j], dets[i].bbox, dets[i].prob[j] * 100, counter);
                        DetectData y(cv::Rect2f(dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h)
                                , names[j], dets[i].prob[j] * 100);
                        res_name.push_back(y);
                        break;
                    }
                }
                if (class1 >= 0) {
                    int width = im.h * .006;

                    /*
                       if(0){
                       width = pow(prob, 1./2.)*10+1;
                       alphabet = 0;
                       }
                     */

                    //printf("%d %s: %.0f%%\n", i, names[class], prob*100);
                    int offset = class1 * 123457 % classes;
                    float red = get_color(2, offset, classes);
                    float green = get_color(1, offset, classes);
                    float blue = get_color(0, offset, classes);
                    float rgb[3];

                    //width = prob*20+2;

                    rgb[0] = red;
                    rgb[1] = green;
                    rgb[2] = blue;
                    box b = dets[i].bbox;
                    //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

                    int left = (b.x - b.w / 2.) * im.w;
                    int right = (b.x + b.w / 2.) * im.w;
                    int top = (b.y - b.h / 2.) * im.h;
                    int bot = (b.y + b.h / 2.) * im.h;

                    if (left < 0) left = 0;
                    if (right > im.w - 1) right = im.w - 1;
                    if (top < 0) top = 0;
                    if (bot > im.h - 1) bot = im.h - 1;

                    draw_box_width(im, left, top, right, bot, width, red, green, blue);
                    if (alphabets) {
                        image label = get_label(alphabets, labelstr, (im.h * .03));
                        draw_label(im, top + width, left, label, rgb);
                        free_image(label);
                    }
                    if (dets[i].mask) {
                        image mask = float_to_image(14, 14, 1, dets[i].mask);
                        image resized_mask = resize_image(mask, b.w * im.w, b.h * im.h);
                        image tmask = threshold_image(resized_mask, .5);
                        embed_image(tmask, im, left, top);
                        free_image(mask);
                        free_image(resized_mask);
                        free_image(tmask);
                    }
                }
            }
        }
    }
}