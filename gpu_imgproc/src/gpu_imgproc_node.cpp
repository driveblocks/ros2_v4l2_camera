#include <rclcpp/rclcpp.hpp>
#include "gpu_imgproc/gpu_imgproc.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gpu_imgproc::GpuImgProc>());
    rclcpp::shutdown();
    return 0;
}