#include "MvCameraControl.h"
#include <cameras.h>
#include <string>
#include <opencv2/opencv.hpp>
int main() {
    std::string cfg_path = "/home/bupt-rc/Code/opencv/2024/cameras_v2/config.yaml";
    cv::Mat frame;
    cameras cam(cfg_path);
    CAMERAS_CHECK(cam.open(), "camera open error");
    CAMERAS_CHECK(cam.start(), "camera start error");
    cv::namedWindow(cam.cam_name_, cv::WINDOW_NORMAL);

    while (true) {
        frame = cam.get_frame();
        if (frame.empty())
            continue;
        cv::imshow(cam.cam_name_, frame);
        if(cv::waitKey(1) == 'q')
            break;
    }
    CAMERAS_CHECK(cam.stop(), "camera stop error");
    cv::destroyAllWindows();
    return 0;
}
