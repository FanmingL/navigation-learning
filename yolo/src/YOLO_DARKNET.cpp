//
// Created by Fanming Luo on 2018/9/23.
//

#include "YOLO_DARKNET.h"

void YOLO_DARKNET::load_alphabet_new()
{
    int i, j;
    const int nsize = 8;
    alphabets = (image **)calloc(nsize, sizeof(image));
    for(j = 0; j < nsize; ++j){
        alphabets[j] = (image *)calloc(128, sizeof(image));
        for(i = 32; i < 127; ++i){
            char buff[256];
            sprintf(buff, "%s/%d_%d.png",(char*)alphabetPath.c_str(), i, j);
            alphabets[j][i] = load_image_color(buff, 0, 0);
        }
    }
}

YOLO_DARKNET::YOLO_DARKNET() :
basePath(PATH), configPath(basePath+"/config/yolov3.cfg"), weightPath(basePath+"/data/yolov3.weights"),
    namesPath(basePath+"/data/coco.names"), alphabetPath(basePath+"/data/labels")
    ,counter(0)
{
    names = get_labels((char*)namesPath.c_str());
    net = load_network((char*)configPath.c_str(), (char*)weightPath.c_str(), 0);
    set_batch_network(net, 1);
    // resize_network(net, 960, 960);
    load_alphabet_new();
}

void YOLO_DARKNET::yoloProcess(cv::Mat &in, cv::Mat &res, std::vector<YOLO_OUT>& anchor,
        float threshold, float hier, float nms) {
    int nboxes = 0;
    image _tmp_image = mat_to_image(in);
    image _sized = letterbox_image(_tmp_image, net->w, net->h);

    layer *l = &(net->layers[net->n-1]);
    network_predict(net, _sized.data);
    detection *dets = get_network_boxes(net, _tmp_image.w, _tmp_image.h, threshold, hier, nullptr, 1, &nboxes);
    if(nms)do_nms_sort(dets, nboxes, l->classes, nms);
    draw_detections_new(_tmp_image, dets, nboxes, threshold, l->classes, anchor);
    res = image_to_mat(_tmp_image);
    free_detections(dets, nboxes);
    free_image(_sized);
    free_image(_tmp_image);
    counter++;
}

void YOLO_DARKNET::printDetection(std::vector<YOLO_OUT> &detection_ins) {
    for (auto const &item : detection_ins)
    {
        std::cout<<"name is "<<item.name
        <<"\nbox is ("<<item.bbox.x<<", "<<item.bbox.y<<", "
        <<item.bbox.w<<", "<<item.bbox.h<<") "
        <<"\n prob is "<<item.prob<<"\n\n";
    }

}

void YOLO_DARKNET::draw_detections_new(image im, detection *dets, int num,
        float thresh, int classes, std::vector<YOLO_OUT> &res_name)
{
    int i,j;
    res_name.clear();
    for(i = 0; i < num; ++i){
        char labelstr[4096] = {0};
        int class1 = -1;
        for(j = 0; j < classes; ++j){
            if (dets[i].prob[j] > thresh){
                if (class1 < 0) {
                    strcat(labelstr, names[j]);
                    class1 = j;
                } else {
                    strcat(labelstr, ", ");
                    strcat(labelstr, names[j]);
                }
                printf("%s: %.0f%%\n", names[j], dets[i].prob[j]*100);
                YOLO_OUT y(names[j], dets[i].bbox, dets[i].prob[j]*100, counter);
                res_name.push_back(y);
                break;
            }
        }
        if(class1 >= 0){
            int width = im.h * .006;

            /*
               if(0){
               width = pow(prob, 1./2.)*10+1;
               alphabet = 0;
               }
             */

            //printf("%d %s: %.0f%%\n", i, names[class], prob*100);
            int offset = class1*123457 % classes;
            float red = get_color(2,offset,classes);
            float green = get_color(1,offset,classes);
            float blue = get_color(0,offset,classes);
            float rgb[3];

            //width = prob*20+2;

            rgb[0] = red;
            rgb[1] = green;
            rgb[2] = blue;
            box b = dets[i].bbox;
            //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

            int left  = (b.x-b.w/2.)*im.w;
            int right = (b.x+b.w/2.)*im.w;
            int top   = (b.y-b.h/2.)*im.h;
            int bot   = (b.y+b.h/2.)*im.h;

            if(left < 0) left = 0;
            if(right > im.w-1) right = im.w-1;
            if(top < 0) top = 0;
            if(bot > im.h-1) bot = im.h-1;

            draw_box_width(im, left, top, right, bot, width, red, green, blue);
            if (alphabets) {
                image label = get_label(alphabets, labelstr, (im.h*.03));
                draw_label(im, top + width, left, label, rgb);
                free_image(label);
            }
            if (dets[i].mask){
                image mask = float_to_image(14, 14, 1, dets[i].mask);
                image resized_mask = resize_image(mask, b.w*im.w, b.h*im.h);
                image tmask = threshold_image(resized_mask, .5);
                embed_image(tmask, im, left, top);
                free_image(mask);
                free_image(resized_mask);
                free_image(tmask);
            }
        }
    }
}

std::ostream & operator << (std::ostream &out,
        YOLO_DARKNET::YOLO_OUT &item) {
        out<<item.frame_index<<", "<<item.name
        <<", "<<item.bbox.x<<", "<<item.bbox.y<<", "
        <<item.bbox.w<<", "<<item.bbox.h<<", "
        <<item.prob;
    return out;
    }


void YOLO_DARKNET::videoProcess(const char *_in_path, const char *_out_path) {
    std::string in_path(basePath+_in_path), out_path(basePath+_out_path);

    cv::VideoWriter videoWriter(out_path, -1, 30, cv::Size(960, 960));
    cv::VideoCapture cap(in_path);
    cv::Mat m;
    cv::Rect rect(1280-960, 0, 960, 960);
    std::vector<YOLO_OUT> res;

    std::ofstream outfile;
    outfile.open(basePath+"/data/res-video.txt");
    if (!outfile)std::cout<<"Error"<<std::endl;
    double t = what_time_is_it_now();
    for (;;)
    {

        cap >> m;
        if (m.empty())break;
        cv::Mat resized(m, rect);
        yoloProcess(resized, resized, res, 0.5, 0.5, 0.4);
        videoWriter << resized;
        for (auto &item : res)
            outfile << item <<std::endl << std::endl;
        std::cout<<counter<<", "<<1./(what_time_is_it_now()-t)<<std::endl;
        t = what_time_is_it_now();

    }
    outfile.close();
}

