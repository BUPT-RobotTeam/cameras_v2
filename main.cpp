#include "MvCameraControl.h"
#include <cameras.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <thread>

void thread_function(std::string& cfg_path) {
    try {
        cv::Mat frame;
        cameras cam(cfg_path);
        CAMERAS_CHECK(cam.open(), "camera open error");
        CAMERAS_CHECK(cam.start(), "camera start error");
        cv::namedWindow(cam.get_cam_name(), cv::WINDOW_NORMAL);
        while (true) {
            frame = cam.get_frame();
            if (frame.empty())
                continue;
            std::cout << cam.get_cam_name() << " fps: " << cam.get_fps() << std::endl;
            cv::imshow(cam.get_cam_name(), frame);
            if (cv::waitKey(1) == 'q')
                break;
        }
        CAMERAS_CHECK(cam.stop(), "camera stop error");
    }
    catch (const std::exception& e) {
        std::cout << "[BUPT_RC] Exception caught: " << e.what() << std::endl;
    }
}

int main() {

    const int camera_num = 2;
    std::thread threads[camera_num];
    std::string cfg_path = "/home/bupt-rc/Code/opencv/2024/cameras_v2/config.yaml";
    
    for (int i = 0; i < camera_num; ++i) {
        threads[i] = std::thread(thread_function, std::ref(cfg_path));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    for (int i = 0; i < camera_num; ++i)
        threads[i].join();

    cv::destroyAllWindows();
    return 0;
}
