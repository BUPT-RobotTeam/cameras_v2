#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include "hikcam.hpp"

#define CAMERAS_CHECK(state, str) \
    do { \
        if (!(state)) { \
            std::cerr << (str) << std::endl; \
            return -1; \
        } \
    } while (false)

struct camera_cfg {
    std::string cfg_cam_name_;
    std::string cfg_cam_type_;
    int cfg_frame_height_;
    int cfg_frame_width_;
    int cfg_cam_fps_;
};

class cameras {
public:
    cameras(std::string cfg_path);
    cameras(std::string cfg_path, std::string cam_name);
    ~cameras();

    bool open();
    bool start();
    bool stop();
    cv::Mat get_frame();
    double get_depth(int x, int y);
    double get_fps();
    void test();

private:
    // 配置设置
    bool get_cfg(std::string cfg_path);
    bool cfg_is_usefule(camera_cfg& cfg);
    bool cfg_cam_type_is_useful(std::string& cam_type);
    bool cfg_cam_frame_range_is_useful(std::string& cam_type, int frame_width, int frame_height);
    bool cfg_cam_fps_is_useful(std::string& cam_type, int cam_fps);

    // 自动检测摄像头
    bool auto_detect_cam();
    bool cam_is_accessible_usb();
    bool cam_is_accessible_hik();
    bool cam_is_accessible_realsense();

    // 打印相机信息
    void print_cam_info(bool state);

public:
    static std::vector<std::string> cam_type_list;
    static std::map<std::string, std::vector<int>> cam_frame_range;
    static std::map<std::string, std::vector<int>> cam_fps_range;

private:
    // 相机类型相关
    hikcam cam_hik_;
    cv::VideoWriter cam_usb_;
    rs2::pipeline cam_cfg_realsense_pipe_;
    std::string cam_type_;
    std::string cam_name_;

    // 深度相机配置
    rs2::config cam_cfg_realsense_;

    // 图像帧数据相关
    cv::Mat frame_;
    rs2::frameset frame_realsense_;
    int frame_height_;
    int frame_width_;
    int cam_fps_;

    // 配置文件相关
    std::vector<camera_cfg*> cfg_vector;

    // 计算帧数
    uint32_t intv_[10]; 
    uint32_t idx_;
    decltype(std::chrono::high_resolution_clock::now()) last_read_;
};
