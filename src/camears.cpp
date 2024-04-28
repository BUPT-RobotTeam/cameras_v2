#include "cameras.h"

// 可用摄像头列表
std::vector<std::string> cameras::cam_type_list = {
    "usb_camera",                       // usb 摄像头
    "hik_camera_cs016_10uc",            // 海康工业摄像头(新购入的)
    "hik_camera_ca013_21uc",            // 海康工业摄像头(旧的)
    "realsense_camera_d435i",           // 深度摄像头
};

// 摄像头视频帧大小范围
std::map<std::string, std::vector<int>> cameras::cam_frame_range {
    {"usb_camera", {0, 0, 1920, 1080}},
    {"hik_camera_cs016_10uc", {0, 0, 1440, 1080}},
    {"hik_camera_ca013_21uc", {0, 0, 1280, 1024}},
    {"realsense_camera_d435i", {0, 0, 1920, 1080}},
};

// 摄像头视频帧数大小范围
std::map<std::string, std::vector<int>> cameras::cam_fps_range {
    {"usb_camera", {0, 30}},
    {"hik_camera_cs016_10uc", {0, 100}},
    {"hik_camera_ca013_21uc", {0, 50}},
    {"realsense_camera_d435i", {0, 30}},
};

cameras::cameras(std::string cfg_path) : intv_{0}, last_read_(std::chrono::high_resolution_clock::now()), idx_(0){
    std::fill_n(this->intv_, sizeof(this->intv_) / sizeof(this->intv_[0]), 0x7f7f7f7f);
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_FATAL);                   // 设置opencv打印日志的等级, 不再打印warn信息
    try {
        this->get_cfg(cfg_path);
    }
    catch (const std::exception& e) {
        std::cout << "[BUPT_RC]Exception caught: " << e.what() << std::endl;
    }
}

cameras::~cameras() {
    for (camera_cfg* cam_cfg : this->cfg_vector) 
        delete cam_cfg;
    std::cout << "Resources have been released" << std::endl;
}

bool cameras::get_cfg(std::string yaml_path) {
    bool cfg_is_useful = true;  
    do {
        try {
            //----------------------------------------
            // 打开文件
            auto yaml = YAML::LoadAllFromFile(yaml_path);

            //----------------------------------------
            // 把配置加载到向量中
            for (const auto& cfg : yaml) {
                camera_cfg* cam_cfg = new camera_cfg;
                cam_cfg->cfg_cam_name_ = cfg["camera_name"].as<std::string>();
                cam_cfg->cfg_cam_type_ = cfg["camera_type"].as<std::string>();
                cam_cfg->cfg_cam_fps_ = cfg["camera_fps"].as<int>();
                cam_cfg->cfg_frame_width_ = cfg["frame_width"].as<int>();
                cam_cfg->cfg_frame_height_ = cfg["frame_height"].as<int>();
                
                if (!cfg_is_usefule(*cam_cfg))
                     throw std::runtime_error("camera configuration file error");

                this->cfg_vector.push_back(cam_cfg);
            }
        }
        catch (const YAML::Exception& e) {
            std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
            cfg_is_useful = false;
            break;
        }
    } while(0);

    return cfg_is_useful;
}

bool cameras::cfg_is_usefule(camera_cfg& cfg) {
    bool cfg_is_usefule = true; 
    if (cfg_cam_type_is_useful(cfg.cfg_cam_type_)) {
        if (!cfg_cam_frame_range_is_useful(cfg.cfg_cam_type_, cfg.cfg_frame_width_, cfg.cfg_frame_height_))
            cfg_is_usefule = false;
        else if (!cfg_cam_fps_is_useful(cfg.cfg_cam_type_, cfg.cfg_cam_fps_)) 
            cfg_is_usefule = false;
        else 
            cfg_is_usefule = true;
    }
    else {
        cfg_is_usefule = false;
    }
    return cfg_is_usefule;
}

bool cameras::cfg_cam_type_is_useful(std::string& cam_type) {
    bool cam_type_is_useful = false;
    for (const auto& type : cameras::cam_type_list) {
        if (cam_type == type) {
            cam_type_is_useful = true;
            break;
        }
    }
    return cam_type_is_useful;
}

bool cameras::cfg_cam_frame_range_is_useful(std::string& cam_type, int frame_width, int frame_height) {
    bool cam_frame_range_is_useful = true;
    std::vector<int> range = cameras::cam_frame_range[cam_type];

    if (frame_width < range[0]) 
        cam_frame_range_is_useful = false;
    else if (frame_width > range[2])
        cam_frame_range_is_useful = false;
    else if (frame_height < range[1])
        cam_frame_range_is_useful = false; 
    else if (frame_height > range[3])
        cam_frame_range_is_useful = false; 
    else
        cam_frame_range_is_useful = true; 
        
    return cam_frame_range_is_useful;
}

bool cameras::cfg_cam_fps_is_useful(std::string& cam_type, int cam_fps) {
    std::vector<int> range = cameras::cam_fps_range[cam_type];
    return cam_fps < range[0] ? false : (cam_fps > range[1] ? false : true);
}

void cameras::test() {
    for (camera_cfg* cam_cfg : this->cfg_vector) {
        std::cout << "camera name   : " << cam_cfg->cfg_cam_name_ << std::endl;
        std::cout << "camera type   : " << cam_cfg->cfg_cam_type_ << std::endl;
        std::cout << "camera fps    : " << cam_cfg->cfg_cam_fps_ << std::endl;
        std::cout << "frame widht   : " << cam_cfg->cfg_frame_width_ << std::endl;
        std::cout << "camera name   : " << cam_cfg->cfg_frame_height_ << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
    }
}
