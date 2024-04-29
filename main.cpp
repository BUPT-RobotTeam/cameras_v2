#include "MvCameraControl.h"
#include <cameras.h>
#include <string>
#include <opencv2/opencv.hpp>
int main() {
    std::string cfg_path = "/home/bupt-rc/Code/opencv/2024/cameras_v2/config.yaml";

    cv::Mat frame_0;
    cameras cam_0(cfg_path);
    CAMERAS_CHECK(cam_0.open(), "camera open error");
    CAMERAS_CHECK(cam_0.start(), "camera start error");
    cv::namedWindow(cam_0.cam_name_, cv::WINDOW_NORMAL);

    cv::Mat frame_1;
    cameras cam_1(cfg_path);
    CAMERAS_CHECK(cam_1.open(), "camera open error");
    CAMERAS_CHECK(cam_1.start(), "camera start error");
    cv::namedWindow(cam_1.cam_name_, cv::WINDOW_NORMAL);

    cv::Mat frame_2;
    cameras cam_2(cfg_path);
    CAMERAS_CHECK(cam_2.open(), "camera open error");
    CAMERAS_CHECK(cam_2.start(), "camera start error");
    cv::namedWindow(cam_2.cam_name_, cv::WINDOW_NORMAL);

    /*
    cv::Mat frame_3;
    cameras cam_3(cfg_path);
    CAMERAS_CHECK(cam_3.open(), "camera open error");
    CAMERAS_CHECK(cam_3.start(), "camera start error");
    cv::namedWindow(cam_3.cam_name_, cv::WINDOW_NORMAL);
    */

    while (true) {
        frame_0 = cam_0.get_frame();
        frame_1 = cam_1.get_frame();
        frame_2 = cam_2.get_frame();
        // frame_3 = cam_3.get_frame();

        // if (frame_0.empty() || frame_1.empty() || frame_2.empty() || frame_3.empty())
        if (frame_0.empty() || frame_1.empty() || frame_2.empty())
        // if (frame_0.empty() || frame_1.empty())
            continue;

        cv::imshow(cam_0.cam_name_, frame_0);
        cv::imshow(cam_1.cam_name_, frame_1);
        cv::imshow(cam_2.cam_name_, frame_2);
        // cv::imshow(cam_3.cam_name_, frame_3);

        if (cv::waitKey(1) == 'q')
            break;
    }

    CAMERAS_CHECK(cam_0.stop(), "camera stop error");
    CAMERAS_CHECK(cam_1.stop(), "camera stop error");
    CAMERAS_CHECK(cam_2.stop(), "camera stop error");
    // CAMERAS_CHECK(cam_3.stop(), "camera stop error");

    cv::destroyAllWindows();
    return 0;
}
