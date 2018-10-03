//
// Created by erdou on 18-10-3.
//
#include "undisort.h"
int main(int argc, char **argv)
{
    undisort undisort1("/data/RSD-005-13min.mp4", "/data/in_video_undistort.mp4");
    cv::Mat dst;
    while (undisort1.run(dst))
    {
#ifdef SHOW_VIDEO
        cv::imshow("1111",dst);
        auto key = cv::waitKey(1);
        if (key == 'q')break;
#endif
    }
    return 0;
}
