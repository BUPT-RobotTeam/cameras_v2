#include "cameras.h"

// 可用摄像头列表
std::vector<std::string> cameras::cam_type_list = {
    "usb_camera",                       // usb 摄像头
    "hik_camera_mv_cs016_10uc",         // 海康工业摄像头(新购入的)
    "hik_camera_mv_ca013_21uc",         // 海康工业摄像头(旧的)
    "realsense_camera_d435i",           // 深度摄像头
};

// 摄像头视频帧大小范围
std::map<std::string, std::vector<int>> cameras::cam_frame_range {
    {"usb_camera", {0, 0, 1920, 1080}},
    {"hik_camera_mv_cs016_10uc", {0, 0, 1440, 1080}},
    {"hik_camera_mv_ca013_21uc", {0, 0, 1280, 1024}},
    {"realsense_camera_d435i", {0, 0, 1920, 1080}},
};

// 摄像头视频帧数大小范围
std::map<std::string, std::vector<int>> cameras::cam_fps_range {
    {"usb_camera", {0, 30}},
    {"hik_camera_mv_cs016_10uc", {0, 100}},
    {"hik_camera_mv_ca013_21uc", {0, 50}},
    {"realsense_camera_d435i", {0, 30}},
};

cameras::cameras(std::string cfg_path) : intv_{0}, last_read_(std::chrono::high_resolution_clock::now()), idx_(0){  
    std::fill_n(this->intv_, sizeof(this->intv_) / sizeof(this->intv_[0]), 0x7f7f7f7f);
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_FATAL);                   // 设置opencv打印日志的等级, 不再打印warn信息
    this->cam_pos_ = 0;                                                                                        

    try {
        this->get_cfg(cfg_path);
        this->auto_detect_cam();
        this->print_info();
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

bool cameras::open() {
    bool status = true;
    if (this->cam_base_type_ == "usb_camera") {
        this->cam_usb_ = cv::VideoCapture(this->cam_pos_);
        status = this->cam_usb_.isOpened(); 
    }
    else if (this->cam_base_type_ == "hik_camera") {
        status = this->cam_hik_.open(this->cam_pos_, this->frame_width_, this->frame_height_);
    }
    else if (this->cam_base_type_ == "realsense_camera") {
        rs2::context ctx;
        rs2::device_list dev_list = ctx.query_devices(); 
        rs2::device dev = dev_list[this->cam_realsense_pos_]; 
        std::string serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        this->cam_cfg_realsense_.enable_device(serial_number);
        status = true;
    }
    return status;
}

bool cameras::start() {
    bool status = false;
    if (this->cam_base_type_ == "usb_camera") {
        this->cam_usb_.set(cv::CAP_PROP_FRAME_WIDTH, this->frame_width_);
        this->cam_usb_.set(cv::CAP_PROP_FRAME_HEIGHT, this->frame_height_);
        status = true; 
    }
    else if (this->cam_base_type_ == "hik_camera") {
        status = this->cam_hik_.start();
    }
    else if (this->cam_base_type_ == "realsense_camera") {
        this->cam_cfg_realsense_.enable_stream(RS2_STREAM_COLOR, this->frame_width_, this->frame_height_, RS2_FORMAT_RGB8, this->cam_fps_);
        this->cam_cfg_realsense_.enable_stream(RS2_STREAM_DEPTH, this->frame_width_, this->frame_height_, RS2_FORMAT_Z16, this->cam_fps_);
        this->cam_realsense_pipe_.start(this->cam_cfg_realsense_);
        status = true;
    }
    return status;
}
bool cameras::stop() {
    bool status = false;
    if (this->cam_base_type_ == "usb_camera") {
        this->cam_usb_.release();
        status = !this->cam_usb_.isOpened(); 
    }
    else if (this->cam_base_type_ == "hik_camera") {
        status = this->cam_hik_.stop();
    }
    else if (this->cam_base_type_ == "realsense_camera") {
        this->cam_realsense_pipe_.stop();
        status = true;
    }
    return status;
}

cv::Mat cameras::get_frame() {
    if (this->cam_base_type_ == "usb_camera") {
        this->cam_usb_.read(this->frame_);
    }
    else if (this->cam_base_type_ == "hik_camera") {
        this->frame_ = this->cam_hik_.read();
    }
    else if (this->cam_base_type_ == "realsense_camera") {
        this->frame_realsense_ = this->cam_realsense_pipe_.wait_for_frames(); 
        rs2::frame color_frame = this->frame_realsense_.get_color_frame();
        cv::Mat img(cv::Size(this->frame_width_, this->frame_height_), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(img, this->frame_, cv::COLOR_RGB2BGR);
    }

    return this->frame_;
}

void cameras::get_cfg(std::string yaml_path) {
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
        }
    } while(0);

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


void cameras::auto_detect_cam() {
    bool find_useful_cam = false;
    this->cam_usb_pos_ = 0;
    this->cam_hik_pos_ = 0;
    this->cam_realsense_pos_ = 0;

    for (camera_cfg* cfg : this->cfg_vector) {
        auto delimiter_pos = cfg->cfg_cam_type_.find('_') + sizeof("camera");
        auto cam_base_type = cfg->cfg_cam_type_.substr(0, delimiter_pos);

        if (cam_base_type == "usb_camera") {
            find_useful_cam = cam_is_accessible_usb();
        }
        else if (cam_base_type == "hik_camera") {
            find_useful_cam = cam_is_accessible_hik(*cfg);
        }
        else if (cam_base_type == "realsense_camera") {
            find_useful_cam = cam_is_accessible_realsense();
        }
        else {
            find_useful_cam = false;
            break;
        }

        if (find_useful_cam) {
            this->cam_name_ = cfg->cfg_cam_name_;
            this->cam_type_ = cfg->cfg_cam_type_;
            this->cam_fps_ = cfg->cfg_cam_fps_;
            this->frame_width_ = cfg->cfg_frame_width_;
            this->frame_height_ = cfg->cfg_frame_height_;
            this->cam_base_type_ = cam_base_type;
            break;
        }
    }

    if (!find_useful_cam) 
        throw std::runtime_error("No cameras are available on the bus");
}


bool cameras::cam_is_accessible_usb() {
    bool cam_is_accessible = false;

    cv::VideoCapture cap(this->cam_usb_pos_);
    if (cap.isOpened()) {
        cam_is_accessible = true;
        cap.release();
        this->cam_pos_ = this->cam_usb_pos_;
    }
    else {
        cam_usb_pos_ += 2;                  // 这里是V4L2的特性，一个usb摄像头会占用两个video设备节点，很抽象
    }

    return cam_is_accessible;
}


bool cameras::cam_is_accessible_hik(camera_cfg& cfg) {
    //----------------------------------------
    // 枚举设备
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_OK;
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        throw std::runtime_error("MV_CC_EnumDevices fail!");
    }
    
    //----------------------------------------
    // 检测是否可用
    bool cam_is_accessible = false;
    bool cam_is_match = true;
    if (stDeviceList.nDeviceNum > 0) {

        for (; this->cam_hik_pos_ < stDeviceList.nDeviceNum; ++this->cam_hik_pos_) {
            MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[this->cam_hik_pos_]);
            cam_is_accessible = MV_CC_IsDeviceAccessible(stDeviceList.pDeviceInfo[this->cam_hik_pos_], MV_ACCESS_Exclusive);
            MV_CC_DestroyHandle(handle);

            if (cam_is_accessible)
                break;
        }

        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[this->cam_hik_pos_];
        std::string cam_mode_name(reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName));
        std::replace(cam_mode_name.begin(), cam_mode_name.end(), '-', '_');
        std::transform(cam_mode_name.begin(), cam_mode_name.end(), cam_mode_name.begin(), [](unsigned char c) {return std::tolower(c); });

        if (cam_is_accessible) {
            if (!(cfg.cfg_cam_type_.substr(sizeof("hik_camera")) == cam_mode_name)) 
                cam_is_accessible = false;
            else
                this->cam_pos_ = this->cam_hik_pos_;
        }
    }
    return cam_is_accessible;
}

bool cameras::cam_is_accessible_realsense() {
    bool cam_is_accessible = false;

    rs2::context ctx;
    rs2::device_list dev_list = ctx.query_devices(); 

    if (dev_list.size() > 0 && this->cam_realsense_pos_ < dev_list.size()) {
        rs2::device dev = dev_list[this->cam_realsense_pos_]; 
        std::string serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        try {
            rs2::pipeline pipeline;
            rs2::config cfg;
            cfg.enable_device(serial_number);
            pipeline.start(cfg);                                       // start 失败的话就不用 stop了
            pipeline.stop();
            cam_is_accessible = true;
            this->cam_pos_ = this->cam_realsense_pos_;
        } 
        catch (const rs2::error &e) {
            cam_is_accessible = false;
            ++this->cam_realsense_pos_;
        }
    }

    return cam_is_accessible;
}

void cameras::print_info() {
        std::cout << "camera name   : " << this->cam_name_ << std::endl;
        std::cout << "camera type   : " << this->cam_type_ << std::endl;
        std::cout << "camera fps    : " << this->cam_fps_ << std::endl;
        std::cout << "frame widht   : " << this->frame_width_ << std::endl;
        std::cout << "camera name   : " << this->frame_height_ << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
}
