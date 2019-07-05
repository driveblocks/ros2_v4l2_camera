// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "v4l2_camera/v4l2_camera.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace v4l2_camera
{

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  canceled_{false}
{
  // Prepare camera
  auto device = std::string{"/dev/video0"};
  get_parameter("video_device", device);
  camera_ = std::make_shared<V4l2CameraDevice>(device);

  if (!camera_->open()) {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Read parameters and set up callback
  createParameters();

  // Prepare publisher
  if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
  } else {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, "/image_raw");
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      while (rclcpp::ok() && !canceled_.load()) {
        RCLCPP_DEBUG(get_logger(), "Capture...");
        auto img = camera_->capture();
        auto stamp = now();
        if (img->encoding != output_encoding_) {
          img = convert(*img);
        }
        img->header.stamp = stamp;

        if (get_node_options().use_intra_process_comms()) {
          std::stringstream ss;
          ss << "Image message address [PUBLISH]:\t" << img.get();
          RCLCPP_DEBUG(get_logger(), ss.str());
          image_pub_->publish(std::move(img));
        } else {
          auto ci = cinfo_->getCameraInfo();
          if (!checkCameraInfo(*img, ci)) {
            ci = sensor_msgs::msg::CameraInfo{};
            ci.height = img->height;
            ci.width = img->width;
          }

          ci.header.stamp = stamp;

          camera_transport_pub_.publish(*img, ci);
        }
      }
    }
  };
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void V4L2Camera::createParameters()
{
  // Node paramters
  output_encoding_ = declare_parameter("output_encoding", std::string{"rgb8"});

  // Camera info parameters
  auto camera_info_url = std::string{};
  if (get_parameter("camera_info_url", camera_info_url)) {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), std::string{"Invalid camera info URL: "} + camera_info_url);
    }
  }

  // Format parameters
  using ImageSize = std::vector<int64_t>;
  auto image_size = ImageSize{};
  image_size = declare_parameter<ImageSize>("image_size", {640, 480});
  requestImageSize(image_size);

  // Control parameters
  auto toParamName =
    [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

  for (auto const & c : camera_->getControls()) {
    auto name = toParamName(c.name);
    switch (c.type) {
      case ControlType::INT:
        {
          auto value = declare_parameter<int64_t>(name, camera_->getControlValue(c.id));
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::BOOL:
        {
          auto value = declare_parameter<bool>(name, camera_->getControlValue(c.id) != 0);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::MENU:
        {
          // TODO: treating as integer parameter, implement full menu functionality
          auto value = declare_parameter<int64_t>(name, camera_->getControlValue(c.id));
          camera_->setControlValue(c.id, value);
          break;
        }
      default:
        RCLCPP_WARN(get_logger(),
          std::string{"Control type not currently supported: "} + std::to_string(unsigned(c.type)) +
          ", for controle: " + c.name);
        continue;
    }
    control_name_to_id_[name] = c.id;
  }

  // Register callback for parameter value setting
  set_on_parameters_set_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        result.successful &= handleParameter(p);
      }
      return result;
    });
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = std::string{param.get_name()};
  if (control_name_to_id_.find(name) != control_name_to_id_.end()) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return camera_->setControlValue(control_name_to_id_[name], param.as_bool());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return camera_->setControlValue(control_name_to_id_[name], param.as_int());
      default:
        RCLCPP_WARN(get_logger(),
          std::string{"Control parameter type not currently supported: "} +
          std::to_string(unsigned(param.get_type())) +
          ", for parameter: " + param.get_name());
    }
  } else if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "size") {
    return requestImageSize(param.as_integer_array());
  } else if (param.get_name() == "camera_info_url") {
    auto camera_info_url = param.as_string();
    if (cinfo_->validateURL(camera_info_url)) {
      return cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), std::string{"Invalid camera info URL: "} + camera_info_url);
      return false;
    }
  }

  return false;
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() == 2) {
    auto dataFormat = camera_->getCurrentDataFormat();
    // Do not apply if camera already runs at given size
    if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
      return true;
    }
    dataFormat.width = size[0];
    dataFormat.height = size[1];
    return camera_->requestDataFormat(dataFormat);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Invalid image size; expected dimensions: 2, actual: " + std::to_string(size.size()));
    return false;
  }
}

static unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  val = val < 0 ? 0 : val;
  return val > 255 ? 255 : val;
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(
  const unsigned char y, const unsigned char u, const unsigned char v, unsigned char * r,
  unsigned char * g, unsigned char * b)
{
  const int y2 = (int)y;
  const int u2 = (int)u - 128;
  const int v2 = (int)v - 128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

static void yuyv2rgb(unsigned char const * YUV, unsigned char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    y0 = YUV[i + 0];
    u = YUV[i + 1];
    y1 = YUV[i + 2];
    v = YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  RCLCPP_DEBUG(get_logger(),
    std::string{"Coverting: "} + img.encoding + " -> " + output_encoding_);

  // TODO: temporary until cv_bridge and image_proc are available in ROS 2
  if (img.encoding == sensor_msgs::image_encodings::YUV422 &&
    output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
    outImg->width = img.width;
    outImg->height = img.height;
    outImg->step = img.width * 3;
    outImg->encoding = output_encoding_;
    outImg->data.resize(outImg->height * outImg->step);
    for (auto i = 0u; i < outImg->height; ++i) {
      yuyv2rgb(img.data.data() + i * img.step, outImg->data.data() + i * outImg->step,
        outImg->width);
    }
    return outImg;
  } else {
    RCLCPP_WARN_ONCE(get_logger(),
      std::string{"Conversion not supported yet: "} + img.encoding + " -> " + output_encoding_);
    return nullptr;
  }
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

}  // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
