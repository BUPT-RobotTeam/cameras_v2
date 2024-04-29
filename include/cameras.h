#pragma once
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <cstdlib>
#include <regex>
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
    int cfg_is_used_;
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
    void print_info();

private:
    // 配置设置
    void get_cfg(std::string& cfg_path);
    void cfg_update(std::string& cfg_path);
    bool cfg_is_usefule(camera_cfg& cfg);
    bool cfg_cam_type_is_useful(std::string& cam_type);
    bool cfg_cam_frame_range_is_useful(std::string& cam_type, int frame_width, int frame_height);
    bool cfg_cam_fps_is_useful(std::string& cam_type, int cam_fps);

    // 提取摄像头类型信息
    bool cam_is_hik(std::string& cam_type);
    bool cam_is_usb(std::string& cam_type);
    bool cam_is_realsense(std::string& cam_type);

    // 自动检测摄像头
    void auto_detect_cam();
    bool cam_is_accessible_usb();
    bool cam_is_accessible_hik(camera_cfg& cfg);
    bool cam_is_accessible_realsense();

    // 打印相机信息
    void print_cam_info(bool state);

public:
    static std::vector<std::string> cam_type_list;
    static std::map<std::string, std::vector<int>> cam_frame_range;
    static std::map<std::string, std::vector<int>> cam_fps_range;

public:
    // 相机类型相关
    hikcam cam_hik_;
    cv::VideoCapture cam_usb_;
    rs2::pipeline cam_realsense_pipe_;
    std::string cam_type_;
    std::string cam_name_;
    std::string cam_base_type_;

private:
    // 摄像头的位置(如果是第一个摄像头就是0，如果是第二个摄像头就是1, 以此类推)
    int cam_pos_;            
    int cam_hik_pos_;
    std::string cam_realsense_serial_;


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
    std::string cfg_path_;

    // 计算帧数
    uint32_t intv_[10]; 
    uint32_t idx_;
    decltype(std::chrono::high_resolution_clock::now()) last_read_;
};
