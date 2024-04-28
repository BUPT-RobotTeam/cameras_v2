#include <cameras.h>
#include <string>
int main() {
    std::string cfg_path = "/home/bupt-rc/Code/opencv/2024/cameras_v2/config.yaml";
    cameras cam(cfg_path);
    cam.test();
    return 0;
}
