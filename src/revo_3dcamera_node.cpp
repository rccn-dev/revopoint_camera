#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "3DCamera.hpp"

using namespace std::chrono_literals;

namespace revopoint_camera
{

class RevopointCameraNode : public rclcpp::Node
{
public:
  RevopointCameraNode()
  : Node("revopoint_camera")
  {
    serial_ = declare_parameter<std::string>("serial", "");

    depth_mode_ = declare_parameter<std::string>("depth_mode", "");
    rgb_mode_ = declare_parameter<std::string>("rgb_mode", "");

    depth_width_ = declare_parameter<int>("depth_width", 0);
    depth_height_ = declare_parameter<int>("depth_height", 0);
    depth_fps_ = declare_parameter<double>("depth_fps", 0.0);

    rgb_width_ = declare_parameter<int>("rgb_width", 0);
    rgb_height_ = declare_parameter<int>("rgb_height", 0);
    rgb_fps_ = declare_parameter<double>("rgb_fps", 0.0);

    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "revo_depth_optical_frame");
    rgb_frame_id_ = declare_parameter<std::string>("rgb_frame_id", "revo_rgb_optical_frame");

    depth_info_url_ = declare_parameter<std::string>("depth_camera_info_url", "");
    rgb_info_url_ = declare_parameter<std::string>("rgb_camera_info_url", "");

    publish_extrinsics_tf_ = declare_parameter<bool>("publish_extrinsics_tf", true);
    publish_depth_scale_param_ = declare_parameter<bool>("publish_depth_scale_param", true);
    declare_parameter<double>("depth_scale", 1.0);

    depth_auto_exposure_ = declare_parameter<bool>("depth_auto_exposure", false);
    depth_exposure_us_ = declare_parameter<double>("depth_exposure_us", 3000.0);
    depth_frame_time_us_ = declare_parameter<double>("depth_frame_time_us", 7500.0);

    laser_enable_ = declare_parameter<bool>("laser_enable", true);
    laser_luminance_ = declare_parameter<int>("laser_luminance", 100);

    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", false);

    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", rclcpp::SensorDataQoS());
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", rclcpp::SensorDataQoS());
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", rclcpp::SensorDataQoS());
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", rclcpp::SensorDataQoS());
    if (publish_pointcloud_)
    {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    }

    depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "depth", depth_info_url_);
    rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "rgb", rgb_info_url_);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    if (!start_camera())
    {
      RCLCPP_ERROR(get_logger(), "Failed to start camera");
    }
  }

  ~RevopointCameraNode() override
  {
    stop_camera();
  }

private:
  bool start_camera()
  {
    cs::ISystemPtr system = cs::getSystemPtr();
    std::vector<CameraInfo> cameras;
    ERROR_CODE ret = system->queryCameras(cameras);
    if (ret != SUCCESS || cameras.empty())
    {
      RCLCPP_WARN(get_logger(), "No cameras found by queryCameras (ret=%d, count=%zu). Trying direct connect...", ret, cameras.size());
      camera_ = cs::getCameraPtr();
      ret = camera_->connect();
      if (ret != SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Direct connect failed (ret=%d)", ret);
        return false;
      }

      if (!select_streams())
      {
        return false;
      }

      ret = camera_->startStream(depth_stream_info_, rgb_stream_info_, &RevopointCameraNode::frame_pair_callback, this);
      if (ret != SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Start stream failed (ret=%d)", ret);
        return false;
      }

      configure_depth_exposure();
      configure_laser();
      update_camera_info();

      return true;
    }

    CameraInfo selected = cameras[0];
    if (!serial_.empty())
    {
      bool found = false;
      for (const auto & info : cameras)
      {
        if (serial_ == info.serial)
        {
          selected = info;
          found = true;
          break;
        }
      }
      if (!found)
      {
        RCLCPP_WARN(get_logger(), "Serial %s not found, using first camera", serial_.c_str());
      }
    }

    camera_ = cs::getCameraPtr();
    ret = camera_->connect(selected);
    if (ret != SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Camera connect failed (ret=%d)", ret);
      return false;
    }

    if (!select_streams())
    {
      return false;
    }

    ret = camera_->startStream(depth_stream_info_, rgb_stream_info_, &RevopointCameraNode::frame_pair_callback, this);
    if (ret != SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Start stream failed (ret=%d)", ret);
      return false;
    }

    configure_depth_exposure();
    configure_laser();
    update_camera_info();

    return true;
  }

  void stop_camera()
  {
    if (camera_)
    {
      camera_->stopStream();
      camera_->disconnect();
      camera_.reset();
    }
  }

  bool select_streams()
  {
    std::vector<StreamInfo> depth_infos;
    std::vector<StreamInfo> rgb_infos;

    ERROR_CODE ret = camera_->getStreamInfos(STREAM_TYPE_DEPTH, depth_infos);
    if (ret != SUCCESS || depth_infos.empty())
    {
      RCLCPP_ERROR(get_logger(), "Depth stream infos not available (ret=%d)", ret);
      return false;
    }

    ret = camera_->getStreamInfos(STREAM_TYPE_RGB, rgb_infos);
    if (ret != SUCCESS || rgb_infos.empty())
    {
      RCLCPP_ERROR(get_logger(), "RGB stream infos not available (ret=%d)", ret);
      return false;
    }

    int depth_w = depth_width_;
    int depth_h = depth_height_;
    double depth_fps = depth_fps_;
    if (!depth_mode_.empty())
    {
      if (!parse_mode_string(depth_mode_, depth_w, depth_h, depth_fps))
      {
        RCLCPP_WARN(get_logger(), "Invalid depth_mode '%s', falling back to width/height/fps", depth_mode_.c_str());
      }
    }

    if (!pick_stream(depth_infos, STREAM_FORMAT_Z16, depth_w, depth_h, depth_fps, depth_stream_info_))
    {
      RCLCPP_ERROR(get_logger(), "No suitable depth stream found");
      return false;
    }

    int rgb_w = rgb_width_;
    int rgb_h = rgb_height_;
    double rgb_fps = rgb_fps_;
    if (!rgb_mode_.empty())
    {
      if (!parse_mode_string(rgb_mode_, rgb_w, rgb_h, rgb_fps))
      {
        RCLCPP_WARN(get_logger(), "Invalid rgb_mode '%s', falling back to width/height/fps", rgb_mode_.c_str());
      }
    }

    if (!pick_stream(rgb_infos, STREAM_FORMAT_RGB8, rgb_w, rgb_h, rgb_fps, rgb_stream_info_))
    {
      RCLCPP_ERROR(get_logger(), "No suitable RGB stream found");
      return false;
    }

    return true;
  }

  bool pick_stream(
    const std::vector<StreamInfo> & infos,
    STREAM_FORMAT format,
    int width,
    int height,
    double fps,
    StreamInfo & out)
  {
    for (const auto & info : infos)
    {
      if (info.format != format)
      {
        continue;
      }

      if (width > 0 && info.width != width)
      {
        continue;
      }

      if (height > 0 && info.height != height)
      {
        continue;
      }

      if (fps > 0.0 && std::abs(info.fps - fps) > 0.5)
      {
        continue;
      }

      out = info;
      return true;
    }

    out = infos.front();
    return out.format == format;
  }

  bool parse_mode_string(const std::string & mode, int & width, int & height, double & fps)
  {
    int w = 0;
    int h = 0;
    double f = 0.0;
    if (sscanf(mode.c_str(), "%dx%d@%lfHz", &w, &h, &f) == 3 ||
        sscanf(mode.c_str(), "%dx%d@%lf", &w, &h, &f) == 3)
    {
      width = w;
      height = h;
      fps = f;
      return true;
    }
    return false;
  }

  void configure_depth_exposure()
  {
    PropertyExtension auto_exp;
    auto_exp.autoExposureMode = depth_auto_exposure_ ? AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_FIX_FRAMETIME
                             : AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_CLOSE;

    ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, auto_exp);
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to set depth auto exposure mode (ret=%d)", ret);
    }

    if (!depth_auto_exposure_)
    {
      if (depth_frame_time_us_ > 0.0)
      {
        ret = camera_->setProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, static_cast<float>(depth_frame_time_us_));
        if (ret != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to set depth frame time (ret=%d)", ret);
        }
      }

      if (depth_exposure_us_ > 0.0)
      {
        ret = camera_->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, static_cast<float>(depth_exposure_us_));
        if (ret != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to set depth exposure (ret=%d)", ret);
        }
      }
    }
  }

  void configure_laser()
  {
    PropertyExtension pe;
    pe.ledCtrlParam.emLedId = LED_ID::LASER_LED;
    pe.ledCtrlParam.emCtrlType = laser_enable_ ? LED_CTRL_TYPE::ENABLE_LED : LED_CTRL_TYPE::DISABLE_LED;

    ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_LED_CTRL, pe);
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to set laser enable (ret=%d)", ret);
      return;
    }

    if (laser_enable_)
    {
      PropertyExtension lum;
      lum.ledCtrlParam.emLedId = LED_ID::LASER_LED;
      lum.ledCtrlParam.emCtrlType = LED_CTRL_TYPE::LUMINANCE;
      lum.ledCtrlParam.luminance = static_cast<unsigned int>(laser_luminance_);

      ret = camera_->setPropertyExtension(PROPERTY_EXT_LED_CTRL, lum);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set laser luminance (ret=%d)", ret);
      }
    }
  }

  void update_camera_info()
  {
    depth_info_msg_ = make_camera_info(STREAM_TYPE_DEPTH, depth_stream_info_, depth_frame_id_, depth_info_manager_, &depth_intrinsics_);
    rgb_info_msg_ = make_camera_info(STREAM_TYPE_RGB, rgb_stream_info_, rgb_frame_id_, rgb_info_manager_, &rgb_intrinsics_);

    PropertyExtension scale;
    if (camera_->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, scale) == SUCCESS)
    {
      depth_scale_ = scale.depthScale;
    }
    else
    {
      depth_scale_ = 1.0f;
    }

    if (publish_depth_scale_param_)
    {
      this->set_parameter(rclcpp::Parameter("depth_scale", static_cast<double>(depth_scale_)));
    }

    Extrinsics extrinsics;
    if (publish_extrinsics_tf_ && camera_->getExtrinsics(extrinsics) == SUCCESS)
    {
      publish_extrinsics_transform(extrinsics);
    }
    else if (publish_extrinsics_tf_)
    {
      RCLCPP_WARN(get_logger(), "Failed to get extrinsics for TF publishing");
    }
  }

  void publish_extrinsics_transform(const Extrinsics & extrinsics)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = depth_frame_id_;
    tf_msg.child_frame_id = rgb_frame_id_;

    tf_msg.transform.translation.x = extrinsics.translation[0];
    tf_msg.transform.translation.y = extrinsics.translation[1];
    tf_msg.transform.translation.z = extrinsics.translation[2];

    const float * r = extrinsics.rotation;
    tf2::Matrix3x3 m(
      r[0], r[3], r[6],
      r[1], r[4], r[7],
      r[2], r[5], r[8]);
    tf2::Quaternion q;
    m.getRotation(q);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  sensor_msgs::msg::CameraInfo make_camera_info(
    STREAM_TYPE stream,
    const StreamInfo & info,
    const std::string & frame_id,
    const std::shared_ptr<camera_info_manager::CameraInfoManager> & manager,
    Intrinsics * intr_out = nullptr)
  {
    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.header.frame_id = frame_id;
    cam_info.width = info.width;
    cam_info.height = info.height;

    if (manager && manager->isCalibrated())
    {
      cam_info = manager->getCameraInfo();
      cam_info.header.frame_id = frame_id;
      cam_info.width = info.width;
      cam_info.height = info.height;
      return cam_info;
    }

    Intrinsics intr{};
    Distort dist;
    if (camera_->getIntrinsics(stream, intr) != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to get intrinsics for stream %d", stream);
    }
    if (intr_out)
    {
      *intr_out = intr;
    }

    if (camera_->getDistort(stream, dist) != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to get distortion for stream %d", stream);
    }

    cam_info.k.fill(0.0);
    cam_info.k[0] = intr.fx;
    cam_info.k[2] = intr.cx;
    cam_info.k[4] = intr.fy;
    cam_info.k[5] = intr.cy;
    cam_info.k[8] = 1.0;

    cam_info.p.fill(0.0);
    cam_info.p[0] = intr.fx;
    cam_info.p[2] = intr.cx;
    cam_info.p[5] = intr.fy;
    cam_info.p[6] = intr.cy;
    cam_info.p[10] = 1.0;

    cam_info.r.fill(0.0);
    cam_info.r[0] = cam_info.r[4] = cam_info.r[8] = 1.0;

    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.d = {dist.k1, dist.k2, dist.k3, dist.k4, dist.k5};

    return cam_info;
  }

  static void frame_pair_callback(cs::IFramePtr frame_dep, cs::IFramePtr frame_rgb, void * user_data)
  {
    auto * self = static_cast<RevopointCameraNode *>(user_data);
    if (self)
    {
      self->handle_frame_pair(frame_dep, frame_rgb);
    }
  }

  void handle_frame_pair(cs::IFramePtr frame_dep, cs::IFramePtr frame_rgb)
  {
    if (!frame_dep || frame_dep->empty())
    {
      return;
    }

    rclcpp::Time stamp = this->get_clock()->now();

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = depth_frame_id_;
    depth_msg.height = frame_dep->getHeight();
    depth_msg.width = frame_dep->getWidth();
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg.is_bigendian = false;
    depth_msg.step = static_cast<uint32_t>(depth_msg.width * sizeof(uint16_t));

    const std::size_t depth_size = depth_msg.step * depth_msg.height;
    depth_msg.data.resize(depth_size);

    const char * depth_data = frame_dep->getData(FRAME_DATA_FORMAT_Z16);
    if (depth_data)
    {
      std::memcpy(depth_msg.data.data(), depth_data, depth_size);
    }

    sensor_msgs::msg::CameraInfo depth_info = depth_info_msg_;
    depth_info.header.stamp = stamp;

    depth_pub_->publish(depth_msg);
    depth_info_pub_->publish(depth_info);

    if (publish_pointcloud_ && cloud_pub_)
    {
      publish_pointcloud(depth_msg, frame_rgb);
    }

    if (!frame_rgb || frame_rgb->empty())
    {
      return;
    }

    sensor_msgs::msg::Image rgb_msg;
    rgb_msg.header.stamp = stamp;
    rgb_msg.header.frame_id = rgb_frame_id_;
    rgb_msg.height = frame_rgb->getHeight();
    rgb_msg.width = frame_rgb->getWidth();
    rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_msg.is_bigendian = false;
    rgb_msg.step = static_cast<uint32_t>(rgb_msg.width * 3);

    const std::size_t rgb_size = rgb_msg.step * rgb_msg.height;
    rgb_msg.data.resize(rgb_size);

    const char * rgb_data = frame_rgb->getData();
    if (rgb_data)
    {
      std::memcpy(rgb_msg.data.data(), rgb_data, rgb_size);
    }

    sensor_msgs::msg::CameraInfo rgb_info = rgb_info_msg_;
    rgb_info.header.stamp = stamp;

    rgb_pub_->publish(rgb_msg);
    rgb_info_pub_->publish(rgb_info);
  }

  void publish_pointcloud(const sensor_msgs::msg::Image & depth_msg, cs::IFramePtr frame_rgb)
  {
    if (depth_msg.encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
      return;
    }

    const bool has_rgb = frame_rgb && !frame_rgb->empty() &&
      frame_rgb->getWidth() == static_cast<int>(depth_msg.width) &&
      frame_rgb->getHeight() == static_cast<int>(depth_msg.height);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = depth_msg.header.stamp;
    cloud.header.frame_id = depth_frame_id_;
    cloud.height = depth_msg.height;
    cloud.width = depth_msg.width;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    if (has_rgb)
    {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }
    else
    {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(cloud.width * cloud.height);

    const float fx = depth_intrinsics_.fx;
    const float fy = depth_intrinsics_.fy;
    const float cx = depth_intrinsics_.cx;
    const float cy = depth_intrinsics_.cy;

    if (depth_intrinsics_.fx == 0.0f || depth_intrinsics_.fy == 0.0f)
    {
      RCLCPP_WARN(get_logger(), "Invalid depth intrinsics, skipping pointcloud");
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_r;
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_g;
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_b;
    if (has_rgb)
    {
      iter_r = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(cloud, "r");
      iter_g = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(cloud, "g");
      iter_b = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(cloud, "b");
    }

    const uint16_t * depth = reinterpret_cast<const uint16_t *>(depth_msg.data.data());
    const char * rgb_data = has_rgb ? frame_rgb->getData() : nullptr;

    for (unsigned int v = 0; v < depth_msg.height; ++v)
    {
      for (unsigned int u = 0; u < depth_msg.width; ++u, ++iter_x, ++iter_y, ++iter_z)
      {
        const std::size_t idx = v * depth_msg.width + u;
        const uint16_t d = depth[idx];
        if (d == 0)
        {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
          const float z = static_cast<float>(d) * depth_scale_;
          *iter_x = (static_cast<float>(u) - cx) * z / fx;
          *iter_y = (static_cast<float>(v) - cy) * z / fy;
          *iter_z = z;
        }

        if (has_rgb)
        {
          const std::size_t rgb_idx = idx * 3;
          const uint8_t r = static_cast<uint8_t>(rgb_data[rgb_idx + 0]);
          const uint8_t g = static_cast<uint8_t>(rgb_data[rgb_idx + 1]);
          const uint8_t b = static_cast<uint8_t>(rgb_data[rgb_idx + 2]);
          **iter_r = r;
          **iter_g = g;
          **iter_b = b;
          ++(*iter_r);
          ++(*iter_g);
          ++(*iter_b);
        }
      }
    }

    cloud_pub_->publish(cloud);
  }

  std::string serial_;
  std::string depth_mode_;
  std::string rgb_mode_;
  int depth_width_ = 0;
  int depth_height_ = 0;
  double depth_fps_ = 0.0;
  int rgb_width_ = 0;
  int rgb_height_ = 0;
  double rgb_fps_ = 0.0;

  std::string depth_frame_id_;
  std::string rgb_frame_id_;
  std::string depth_info_url_;
  std::string rgb_info_url_;

  bool publish_extrinsics_tf_ = true;
  bool publish_depth_scale_param_ = true;

  bool depth_auto_exposure_ = false;
  double depth_exposure_us_ = 0.0;
  double depth_frame_time_us_ = 0.0;

  bool laser_enable_ = true;
  int laser_luminance_ = 100;

  bool publish_pointcloud_ = false;

  cs::ICameraPtr camera_;
  StreamInfo depth_stream_info_{};
  StreamInfo rgb_stream_info_{};

  float depth_scale_ = 1.0f;

  Intrinsics depth_intrinsics_{};
  Intrinsics rgb_intrinsics_{};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_;

  sensor_msgs::msg::CameraInfo depth_info_msg_;
  sensor_msgs::msg::CameraInfo rgb_info_msg_;
};

}  // namespace revopoint_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<revopoint_camera::RevopointCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
