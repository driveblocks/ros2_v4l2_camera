#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "NvBuffer.h"
#include "NvCudaProc.h"
#include "NvJpegEncoder.h"
#include "NvUtils.h"

class ImageCompressorNode : public rclcpp::Node {
public:
    ImageCompressorNode() : Node("image_compressor_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "input_image", 10, std::bind(&ImageCompressorNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_image", 10);
        
        // Initialize NvJPEGEncoder
        jpeg_encoder_ = NvJPEGEncoder::createJPEGEncoder("jpeg_encoder");
        if (!jpeg_encoder_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create JPEG encoder");
            throw std::runtime_error("Failed to create JPEG encoder");
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Initialize NvBuffer for RGB24 format
        NvBuffer buffer(V4L2_PIX_FMT_YVU420M, msg->width, msg->height, 0);

        // Copy data to the buffer
        memcpy(buffer.planes[0].data, msg->data.data(), msg->data.size());

        // Encode the image
        unsigned char *out_buf = nullptr;
        unsigned long out_buf_size = 0;
        int quality = 75; // Set JPEG quality

        int ret = jpeg_encoder_->encodeFromBuffer(buffer, JCS_YCbCr, &out_buf, out_buf_size, quality);
        if (ret == 0) {
            auto compressed_msg = sensor_msgs::msg::CompressedImage();
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            compressed_msg.data = std::vector<uint8_t>(out_buf, out_buf + out_buf_size);
            publisher_->publish(compressed_msg);

            // Free the allocated memory for the output buffer
            free(out_buf);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode image");
        }

        buffer.unmap();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    NvJPEGEncoder *jpeg_encoder_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCompressorNode>());
    rclcpp::shutdown();
    return 0;
}
