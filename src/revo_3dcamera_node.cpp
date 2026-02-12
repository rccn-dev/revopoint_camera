#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <exception>

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
#include "h/camera.h"
#include "h/system.h"

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

    depth_profile_ = declare_parameter<std::string>("depth_profile", "");
    rgb_profile_ = declare_parameter<std::string>("rgb_profile", "");

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
    depth_gain_ = declare_parameter<double>("depth_gain", 1.0);
    trigger_mode_ = declare_parameter<int>("trigger_mode", static_cast<int>(TRIGGER_MODE_OFF));
    depth_rgb_match_threshold_ = declare_parameter<int>("depth_rgb_match_threshold", 0);
    depth_rgb_match_rgb_offset_ = declare_parameter<int>("depth_rgb_match_rgb_offset", 0);
    depth_rgb_match_force_rgb_after_depth_ = declare_parameter<bool>(
      "depth_rgb_match_force_rgb_after_depth", false);

    initial_reset_ = declare_parameter<bool>("initial_reset", false);
    clear_frame_buffer_ = declare_parameter<bool>("clear_frame_buffer", false);

    laser_enable_ = declare_parameter<bool>("laser_enable", true);
    laser_luminance_ = declare_parameter<int>("laser_luminance", 100);

    ir_led_enable_ = declare_parameter<bool>("ir_led_enable", false);
    ir_led_luminance_ = declare_parameter<int>("ir_led_luminance", 0);
    rgb_led_mode_ = declare_parameter<std::string>("rgb_led_mode", "disable");

    depth_range_min_mm_ = declare_parameter<int>("depth_range_min_mm", 0);
    depth_range_max_mm_ = declare_parameter<int>("depth_range_max_mm", 0);
    algorithm_contrast_ = declare_parameter<int>("algorithm_contrast", 0);

    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", false);
    publish_depth_viz_ = declare_parameter<bool>("publish_depth_viz", false);
    qos_profile_ = declare_parameter<std::string>("qos_profile", "reliable");

    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", get_qos());
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", get_qos());
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", get_qos());
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", get_qos());
    if (publish_depth_viz_)
    {
      depth_viz_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_viz", get_qos());
    }
    if (publish_pointcloud_)
    {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", get_qos());
    }

    depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "depth", depth_info_url_);
    rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "rgb", rgb_info_url_);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Initialize frame monitoring
    frame_timeout_seconds_ = declare_parameter<double>("frame_timeout_seconds", 5.0);
    enable_watchdog_ = declare_parameter<bool>("enable_watchdog", true);
    
    if (!start_camera())
    {
      RCLCPP_ERROR(get_logger(), "Failed to start camera");
    }
    
    // Start watchdog timer if enabled
    if (enable_watchdog_)
    {
      watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() { this->check_stream_health(); });
      RCLCPP_INFO(get_logger(), "Stream watchdog enabled with %.1f second timeout", frame_timeout_seconds_);
    }
  }

  ~RevopointCameraNode() override
  {
    // Disable watchdog before shutdown
    if (watchdog_timer_)
    {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }
    stop_camera();
  }

private:
  /**
   * @brief Get the configured QoS profile for publishers.
   * 
   * Reads the qos_profile_ member variable and returns the appropriate ROS2 QoS.
   * 
   * Supported QoS profiles:
   * - "reliable": Reliable delivery with history depth of 10 (default)
   * - "best_effort": Best-effort delivery (SensorDataQoS)
   * - "sensor_data": Alias for best_effort (SensorDataQoS)
   * 
   * @return rclcpp::QoS The configured QoS profile, defaults to reliable if unknown
   */
  rclcpp::QoS get_qos() const
  {
    // Parse QoS profile string and return appropriate QoS
    if (qos_profile_ == "reliable")
    {
      // Reliable QoS with history depth of 10
      return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    }
    else if (qos_profile_ == "best_effort" || qos_profile_ == "sensor_data")
    {
      // Best effort QoS (SensorDataQoS)
      return rclcpp::SensorDataQoS();
    }
    else
    {
      RCLCPP_WARN_ONCE(get_logger(), 
        "Unknown QoS profile '%s', defaulting to 'reliable'. "
        "Valid options are: 'reliable', 'best_effort', 'sensor_data'", 
        qos_profile_.c_str());
      return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    }
  }

  bool hardware_reset(const CameraInfo * info)
  {
    CSystem * sys = createSystem();
    if (!sys)
    {
      RCLCPP_WARN(get_logger(), "Hardware reset: failed to create C system context");
      return false;
    }

    CameraInfo temp{};
    if (!serial_.empty())
    {
      std::strncpy(temp.serial, serial_.c_str(), sizeof(temp.serial) - 1);
      temp.serial[sizeof(temp.serial) - 1] = '\0';
    }

    CameraInfo * connect_info = info ? const_cast<CameraInfo *>(info) : &temp;
    CCamera * cam = systemConnectCamera(sys, connect_info);
    if (!cam)
    {
      RCLCPP_WARN(get_logger(), "Hardware reset: failed to connect C camera");
      deleteSystem(sys);
      return false;
    }

    ERROR_CODE ret = cameraReset(cam);
    systemDisconnectCamera(sys, cam);
    deleteSystem(sys);

    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Hardware reset request failed (ret=%d)", ret);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Hardware reset request sent");
    return true;
  }

  bool wait_for_stream_infos(int retries, std::chrono::milliseconds delay)
  {
    for (int i = 0; i < retries; ++i)
    {
      std::vector<StreamInfo> depth_infos;
      std::vector<StreamInfo> rgb_infos;
      ERROR_CODE ret_depth = camera_->getStreamInfos(STREAM_TYPE_DEPTH, depth_infos);
      ERROR_CODE ret_rgb = camera_->getStreamInfos(STREAM_TYPE_RGB, rgb_infos);
      if (ret_depth == SUCCESS && ret_rgb == SUCCESS &&
        !depth_infos.empty() && !rgb_infos.empty())
      {
        return true;
      }

      RCLCPP_WARN(get_logger(),
        "Stream infos not ready (depth ret=%d count=%zu, rgb ret=%d count=%zu), retry %d/%d",
        ret_depth, depth_infos.size(), ret_rgb, rgb_infos.size(), i + 1, retries);
      std::this_thread::sleep_for(delay);
    }
    return false;
  }

  bool reconnect_after_restart(const std::string & serial, CameraInfo * connected_info)
  {
    camera_.reset();
    cs::ISystemPtr system = cs::getSystemPtr();

    for (int i = 0; i < 20; ++i)
    {
      std::vector<CameraInfo> cameras;
      ERROR_CODE ret = system->queryCameras(cameras);
      if (ret == SUCCESS && !cameras.empty())
      {
        CameraInfo selected = cameras[0];
        if (!serial.empty())
        {
          for (const auto & info : cameras)
          {
            if (serial == info.serial)
            {
              selected = info;
              break;
            }
          }
        }

        camera_ = cs::getCameraPtr();
        ret = serial.empty() ? camera_->connect() : camera_->connect(selected);
        if (ret == SUCCESS)
        {
          if (connected_info)
          {
            *connected_info = selected;
          }
          return true;
        }
      }

      std::this_thread::sleep_for(500ms);
    }

    camera_ = cs::getCameraPtr();
    ERROR_CODE ret = camera_->connect();
    if (ret == SUCCESS)
    {
      if (connected_info)
      {
        camera_->getInfo(*connected_info);
      }
      return true;
    }

    return false;
  }

  bool restart_and_reconnect(const std::string & serial, CameraInfo * connected_info)
  {
    ERROR_CODE ret = camera_->restart();
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Camera restart failed (ret=%d)", ret);
      return false;
    }

    camera_->disconnect();
    std::this_thread::sleep_for(8s);

    for (int attempt = 0; attempt < 5; ++attempt)
    {
      if (!reconnect_after_restart(serial, connected_info))
      {
        std::this_thread::sleep_for(1s);
        continue;
      }

      CameraInfo info_check{};
      if (camera_->getInfo(info_check) != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Camera connected but info not ready, retry %d/5", attempt + 1);
        camera_->disconnect();
        std::this_thread::sleep_for(1s);
        continue;
      }

      if (!wait_for_stream_infos(10, 500ms))
      {
        RCLCPP_WARN(get_logger(), "Stream infos not ready after reconnect, retry %d/5", attempt + 1);
        camera_->disconnect();
        std::this_thread::sleep_for(1s);
        continue;
      }

      log_camera_info();
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Failed to reconnect and get stream infos after restart");
    return false;
  }

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

      log_camera_info();

      if (initial_reset_)
      {
        RCLCPP_WARN(get_logger(), "Initial reset requested: restarting camera");
        if (!restart_and_reconnect(serial_, nullptr))
        {
          return false;
        }
      }

      if (clear_frame_buffer_)
      {
        PropertyExtension clear;
        clear.streamType = STREAM_TYPE_DEPTH;
        if (camera_->setPropertyExtension(PROPERTY_EXT_CLEAR_FRAME_BUFFER, clear) != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to clear depth frame buffer");
        }
        clear.streamType = STREAM_TYPE_RGB;
        if (camera_->setPropertyExtension(PROPERTY_EXT_CLEAR_FRAME_BUFFER, clear) != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to clear RGB frame buffer");
        }
      }

      if (!initial_reset_ && !wait_for_stream_infos(10, 500ms))
      {
        RCLCPP_ERROR(get_logger(), "Depth/RGB stream infos not available after reconnect");
        return false;
      }

      if (!select_streams())
      {
        return false;
      }

      if (!start_streams())
      {
        RCLCPP_ERROR(get_logger(), "Failed to start streams");
        return false;
      }

      configure_depth_exposure();
      configure_laser();
      configure_leds();
      configure_depth_controls();
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

    log_camera_info();

    if (initial_reset_)
    {
      RCLCPP_WARN(get_logger(), "Initial reset requested: restarting camera");
      if (!restart_and_reconnect(serial_, &selected))
      {
        return false;
      }
    }

    if (clear_frame_buffer_)
    {
      PropertyExtension clear;
      clear.streamType = STREAM_TYPE_DEPTH;
      if (camera_->setPropertyExtension(PROPERTY_EXT_CLEAR_FRAME_BUFFER, clear) != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to clear depth frame buffer");
      }
      clear.streamType = STREAM_TYPE_RGB;
      if (camera_->setPropertyExtension(PROPERTY_EXT_CLEAR_FRAME_BUFFER, clear) != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to clear RGB frame buffer");
      }
    }

    if (!initial_reset_ && !wait_for_stream_infos(10, 500ms))
    {
      RCLCPP_ERROR(get_logger(), "Depth/RGB stream infos not available after reconnect");
      return false;
    }

    if (!select_streams())
    {
      return false;
    }

    if (!start_streams())
    {
      RCLCPP_ERROR(get_logger(), "Failed to start streams");
      return false;
    }

    configure_depth_exposure();
    configure_laser();
    configure_leds();
    configure_depth_controls();
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

    if (!select_depth_stream(depth_infos))
    {
      RCLCPP_ERROR(get_logger(), "No suitable depth stream found");
      return false;
    }

    if (!select_rgb_stream(rgb_infos))
    {
      RCLCPP_ERROR(get_logger(), "No suitable RGB stream found");
      return false;
    }

    log_stream_info("depth", depth_stream_info_);
    log_stream_info("rgb", rgb_stream_info_);

    return true;
  }

  void log_camera_info()
  {
    CameraInfo info;
    if (camera_->getInfo(info) != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to read camera info");
      return;
    }

    last_camera_info_ = info;
    has_camera_info_ = true;

    RCLCPP_INFO(get_logger(), "Camera connected: name=%s serial=%s unique_id=%s fw=%s algo=%s",
      info.name, info.serial, info.uniqueId, info.firmwareVersion, info.algorithmVersion);
  }

  void log_stream_info(const std::string & label, const StreamInfo & info)
  {
    RCLCPP_INFO(get_logger(), "Stream %s: format=%d %dx%d @ %.1f Hz",
      label.c_str(), static_cast<int>(info.format), info.width, info.height, info.fps);
  }

  bool start_streams()
  {
    ERROR_CODE ret = camera_->startStream(depth_stream_info_, rgb_stream_info_, &RevopointCameraNode::frame_pair_callback, this);
    if (ret == SUCCESS)
    {
      return true;
    }

    RCLCPP_WARN(get_logger(), "Paired stream start failed (ret=%d), falling back to sequential start", ret);

    ret = camera_->startStream(STREAM_TYPE_DEPTH, depth_stream_info_, nullptr, nullptr);
    if (ret != SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Depth stream start failed (ret=%d)", ret);
      return false;
    }

    ret = camera_->startStream(STREAM_TYPE_RGB, rgb_stream_info_, nullptr, nullptr);
    if (ret != SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "RGB stream start failed (ret=%d)", ret);
      camera_->stopStream(STREAM_TYPE_DEPTH);
      return false;
    }

    return true;
  }

  bool parse_profile_string(const std::string & profile, int & width, int & height, double & fps)
  {
    int w = 0;
    int h = 0;
    double f = 0.0;
    if (sscanf(profile.c_str(), "%dx%dx%lf", &w, &h, &f) == 3)
    {
      width = w;
      height = h;
      fps = f;
      return true;
    }
    return false;
  }

  bool pick_profile_stream(
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

      if (info.width != width || info.height != height)
      {
        continue;
      }

      // Some devices report 0.0 Hz; treat as "unknown" and accept.
      if (info.fps > 0.0 && fps > 0.0 && std::abs(info.fps - fps) > 0.5)
      {
        continue;
      }

      out = info;
      // Preserve requested fps if device reports 0.
      if (out.fps <= 0.0 && fps > 0.0)
      {
        out.fps = fps;
      }
      return true;
    }

    return false;
  }

  bool select_depth_stream(const std::vector<StreamInfo> & depth_infos)
  {
    log_available_streams("depth", depth_infos);

    if (depth_profile_.empty())
    {
      for (const auto & info : depth_infos)
      {
        if (info.format == STREAM_FORMAT_Z16 && info.width == 1600 && info.height == 1200)
        {
          depth_stream_info_ = info;
          RCLCPP_INFO(get_logger(), "Using device default depth stream %dx%d @ %.1f Hz",
            info.width, info.height, info.fps);
          return true;
        }
      }
      return false;
    }

    if (!depth_profile_.empty())
    {
      int w = 0;
      int h = 0;
      double f = 0.0;
      if (!parse_profile_string(depth_profile_, w, h, f))
      {
        RCLCPP_WARN(get_logger(), "Invalid depth_profile '%s' (expected WxHxF)", depth_profile_.c_str());
      }
      else
      {
        if (depth_profile_ != "1600x1200x6" && depth_profile_ != "800x600x12")
        {
          RCLCPP_WARN(get_logger(), "Depth profile '%s' is not a default option; attempting to set anyway", depth_profile_.c_str());
        }
        if (pick_profile_stream(depth_infos, STREAM_FORMAT_Z16, w, h, f, depth_stream_info_))
        {
          return true;
        }
        RCLCPP_WARN(get_logger(), "Requested depth profile not found, falling back to defaults");
      }
    }

    if (pick_profile_stream(depth_infos, STREAM_FORMAT_Z16, 1600, 1200, 6.0, depth_stream_info_))
    {
      return true;
    }

    if (pick_profile_stream(depth_infos, STREAM_FORMAT_Z16, 800, 600, 12.0, depth_stream_info_))
    {
      return true;
    }

    for (const auto & info : depth_infos)
    {
      if (info.format == STREAM_FORMAT_Z16)
      {
        RCLCPP_WARN(get_logger(), "Falling back to first available depth Z16 stream %dx%d @ %.1f Hz",
          info.width, info.height, info.fps);
        depth_stream_info_ = info;
        return true;
      }
    }

    return false;
  }

  bool select_rgb_stream(const std::vector<StreamInfo> & rgb_infos)
  {
    log_available_streams("rgb", rgb_infos);

    if (rgb_profile_.empty())
    {
      for (const auto & info : rgb_infos)
      {
        if (info.format == STREAM_FORMAT_RGB8)
        {
          rgb_stream_info_ = info;
          RCLCPP_INFO(get_logger(), "Using device default RGB stream %dx%d @ %.1f Hz",
            info.width, info.height, info.fps);
          return true;
        }
      }
      return false;
    }

    if (!rgb_profile_.empty())
    {
      int w = 0;
      int h = 0;
      double f = 0.0;
      if (!parse_profile_string(rgb_profile_, w, h, f))
      {
        RCLCPP_WARN(get_logger(), "Invalid rgb_profile '%s' (expected WxHxF)", rgb_profile_.c_str());
      }
      else
      {
        if (rgb_profile_ != "1600x1200x20")
        {
          RCLCPP_WARN(get_logger(), "RGB profile '%s' is not a default option; attempting to set anyway", rgb_profile_.c_str());
        }
        if (pick_profile_stream(rgb_infos, STREAM_FORMAT_RGB8, w, h, f, rgb_stream_info_))
        {
          return true;
        }
        RCLCPP_WARN(get_logger(), "Requested RGB profile not found, falling back to defaults");
      }
    }

    if (pick_profile_stream(rgb_infos, STREAM_FORMAT_RGB8, 1600, 1200, 20.0, rgb_stream_info_))
    {
      return true;
    }

    for (const auto & info : rgb_infos)
    {
      if (info.format == STREAM_FORMAT_RGB8)
      {
        RCLCPP_WARN(get_logger(), "Falling back to first available RGB8 stream %dx%d @ %.1f Hz",
          info.width, info.height, info.fps);
        rgb_stream_info_ = info;
        return true;
      }
    }

    return false;
  }

  void log_available_streams(const std::string & label, const std::vector<StreamInfo> & infos)
  {
    for (const auto & info : infos)
    {
      RCLCPP_INFO(get_logger(), "Available %s stream: format=%d %dx%d @ %.1f Hz",
        label.c_str(), static_cast<int>(info.format), info.width, info.height, info.fps);
    }
  }

  void configure_depth_exposure()
  {
    PropertyExtension auto_exp;
    auto_exp.autoExposureMode = depth_auto_exposure_ ? AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_FIX_FRAMETIME
                             : AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_CLOSE;

    ERROR_CODE ret = SUCCESS;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      ret = camera_->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, auto_exp);
      if (ret == SUCCESS)
      {
        break;
      }
      std::this_thread::sleep_for(200ms);
    }
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to set depth auto exposure mode (ret=%d)", ret);
    }
    else
    {
      PropertyExtension auto_exp_check;
      if (camera_->getPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, auto_exp_check) == SUCCESS)
      {
        if (auto_exp_check.autoExposureMode != auto_exp.autoExposureMode)
        {
          RCLCPP_WARN(get_logger(), "Depth auto exposure mismatch: requested=%d actual=%d",
            static_cast<int>(auto_exp.autoExposureMode),
            static_cast<int>(auto_exp_check.autoExposureMode));
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Depth auto exposure set to %d", static_cast<int>(auto_exp.autoExposureMode));
        }
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Failed to read back depth auto exposure mode");
      }
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
        else
        {
          float frame_time_check = 0.0f;
          if (camera_->getProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, frame_time_check) == SUCCESS)
          {
            if (std::fabs(frame_time_check - static_cast<float>(depth_frame_time_us_)) > 1.0f)
            {
              RCLCPP_WARN(get_logger(), "Depth frame time mismatch: requested=%.1f actual=%.1f",
                depth_frame_time_us_, frame_time_check);
            }
            else
            {
              RCLCPP_INFO(get_logger(), "Depth frame time set to %.1f", frame_time_check);
            }
          }
          else
          {
            RCLCPP_WARN(get_logger(), "Failed to read back depth frame time");
          }
        }
      }

      if (depth_exposure_us_ > 0.0)
      {
        ret = camera_->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, static_cast<float>(depth_exposure_us_));
        if (ret != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to set depth exposure (ret=%d)", ret);
        }
        else
        {
          float exposure_check = 0.0f;
          if (camera_->getProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, exposure_check) == SUCCESS)
          {
            if (std::fabs(exposure_check - static_cast<float>(depth_exposure_us_)) > 1.0f)
            {
              RCLCPP_WARN(get_logger(), "Depth exposure mismatch: requested=%.1f actual=%.1f",
                depth_exposure_us_, exposure_check);
            }
            else
            {
              RCLCPP_INFO(get_logger(), "Depth exposure set to %.1f", exposure_check);
            }
          }
          else
          {
            RCLCPP_WARN(get_logger(), "Failed to read back depth exposure");
          }
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

  void configure_leds()
  {
    if (ir_led_enable_ || ir_led_luminance_ > 0)
    {
      PropertyExtension ir_led;
      ir_led.ledCtrlParam.emLedId = LED_ID::IR_LED;
      ir_led.ledCtrlParam.emCtrlType = ir_led_enable_ ? LED_CTRL_TYPE::ENABLE_LED : LED_CTRL_TYPE::DISABLE_LED;

      ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_LED_CTRL, ir_led);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set IR LED enable (ret=%d)", ret);
      }

      if (ir_led_luminance_ > 0)
      {
        PropertyExtension ir_lum;
        ir_lum.ledCtrlParam.emLedId = LED_ID::IR_LED;
        ir_lum.ledCtrlParam.emCtrlType = LED_CTRL_TYPE::LUMINANCE;
        ir_lum.ledCtrlParam.luminance = static_cast<unsigned int>(ir_led_luminance_);
        ret = camera_->setPropertyExtension(PROPERTY_EXT_LED_CTRL, ir_lum);
        if (ret != SUCCESS)
        {
          RCLCPP_WARN(get_logger(), "Failed to set IR LED luminance (ret=%d)", ret);
        }
      }
    }

    if (!rgb_led_mode_.empty())
    {
      LED_CTRL_TYPE ctrl = LED_CTRL_TYPE::DISABLE_LED;
      if (rgb_led_mode_ == "enable")
        ctrl = LED_CTRL_TYPE::ENABLE_LED;
      else if (rgb_led_mode_ == "often_bright")
        ctrl = LED_CTRL_TYPE::OFTEN_BRIGHT_LED;
      else if (rgb_led_mode_ == "twinkle")
        ctrl = LED_CTRL_TYPE::TWINKLE_LED;
      else if (rgb_led_mode_ != "disable")
        RCLCPP_WARN(get_logger(), "Unknown rgb_led_mode '%s'", rgb_led_mode_.c_str());

      PropertyExtension rgb_led;
      rgb_led.ledCtrlParam.emLedId = LED_ID::RGB_LED;
      rgb_led.ledCtrlParam.emCtrlType = ctrl;

      ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_LED_CTRL, rgb_led);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set RGB LED mode (ret=%d)", ret);
      }
    }
  }

  void configure_depth_controls()
  {
    if (depth_gain_ > 0.0)
    {
      ERROR_CODE ret = camera_->setProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, static_cast<float>(depth_gain_));
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set depth gain (ret=%d)", ret);
      }
    }

    if (trigger_mode_ >= 0)
    {
      PropertyExtension trigger;
      trigger.triggerMode = static_cast<TRIGGER_MODE>(trigger_mode_);
      ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_TRIGGER_MODE, trigger);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set trigger mode (ret=%d)", ret);
      }
    }

    if (depth_range_min_mm_ > 0 || depth_range_max_mm_ > 0)
    {
      PropertyExtension range;
      range.depthRange.min = depth_range_min_mm_;
      range.depthRange.max = depth_range_max_mm_;
      ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, range);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set depth range (ret=%d)", ret);
      }
    }

    if (algorithm_contrast_ > 0)
    {
      PropertyExtension contrast;
      contrast.algorithmContrast = algorithm_contrast_;
      ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_CONTRAST_MIN, contrast);
      if (ret != SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Failed to set algorithm contrast (ret=%d)", ret);
      }
    }

    PropertyExtension match;
    match.depthRgbMatchParam.iDifThreshold = depth_rgb_match_threshold_;
    match.depthRgbMatchParam.iRgbOffset = depth_rgb_match_rgb_offset_;
    match.depthRgbMatchParam.bMakeSureRgbIsAfterDepth = depth_rgb_match_force_rgb_after_depth_;
    ERROR_CODE ret = camera_->setPropertyExtension(PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM, match);
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to set depth/rgb match params (ret=%d)", ret);
    }
  }

  void update_camera_info()
  {
    depth_info_msg_ = make_camera_info(STREAM_TYPE_DEPTH, depth_stream_info_, depth_frame_id_, depth_info_manager_, &depth_intrinsics_);
    rgb_info_msg_ = make_camera_info(STREAM_TYPE_RGB, rgb_stream_info_, rgb_frame_id_, rgb_info_manager_, &rgb_intrinsics_);

    PropertyExtension scale;
    if (camera_->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, scale) == SUCCESS)
    {
      // SDK depthScale is in mm per unit (e.g., 0.1 means 0.1mm per depth unit)
      depth_scale_sdk_ = scale.depthScale;  // Store SDK scale in mm/unit
      depth_scale_ = scale.depthScale * 0.001f;  // Convert to meters for ROS
      
      RCLCPP_INFO(get_logger(), "Depth scale from SDK: %.6f mm/unit -> %.6f m/unit", 
        depth_scale_sdk_, depth_scale_);
    }
    else
    {
      // Fallback: Common Revopoint value is 0.1mm per unit
      depth_scale_sdk_ = 0.1f;
      depth_scale_ = 0.0001f; 
      RCLCPP_WARN(get_logger(), "Failed to read depth scale from SDK. Using default 0.1 mm/unit");
    }

    if (publish_depth_scale_param_)
    {
      this->set_parameter(rclcpp::Parameter("depth_scale", static_cast<double>(depth_scale_)));
    }

    Extrinsics extrinsics;
    if (camera_->getExtrinsics(extrinsics) == SUCCESS)
    {
      extrinsics_ = extrinsics;
      has_extrinsics_ = true;
      
      RCLCPP_INFO(get_logger(), "Extrinsics: R=[%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f] T=[%.6f %.6f %.6f]",
        extrinsics.rotation[0], extrinsics.rotation[3], extrinsics.rotation[6],
        extrinsics.rotation[1], extrinsics.rotation[4], extrinsics.rotation[7],
        extrinsics.rotation[2], extrinsics.rotation[5], extrinsics.rotation[8],
        extrinsics.translation[0], extrinsics.translation[1], extrinsics.translation[2]);
      
      RCLCPP_INFO(get_logger(), "Depth intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
        depth_intrinsics_.fx, depth_intrinsics_.fy, depth_intrinsics_.cx, depth_intrinsics_.cy);
      RCLCPP_INFO(get_logger(), "RGB intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
        rgb_intrinsics_.fx, rgb_intrinsics_.fy, rgb_intrinsics_.cx, rgb_intrinsics_.cy);
      
      if (publish_extrinsics_tf_)
      {
        publish_extrinsics_transform(extrinsics);
      }
    }
    else
    {
      has_extrinsics_ = false;
      if (publish_extrinsics_tf_)
      {
        RCLCPP_WARN(get_logger(), "Failed to get extrinsics");
      }
    }
  }

  void check_stream_health()
  {
    auto now = std::chrono::steady_clock::now();
    auto last_frame_time = last_frame_time_.load();
    
    // Skip check if we haven't received any frames yet (during startup)
    if (last_frame_time == std::chrono::steady_clock::time_point{})
    {
      return;
    }
    
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_frame_time).count();
    
    if (elapsed > frame_timeout_seconds_)
    {
      RCLCPP_ERROR(get_logger(), 
        "Stream timeout! No frames received for %ld seconds. Camera may have failed.", 
        elapsed);
      
      // Log frame statistics
      auto total_frames = frame_count_.load();
      auto error_count = error_count_.load();
      RCLCPP_ERROR(get_logger(), 
        "Frame statistics - Total: %lu, Errors: %lu, Last frame: %ld seconds ago",
        total_frames, error_count, elapsed);
    }
    else if (elapsed > frame_timeout_seconds_ / 2)
    {
      // Warning when we're halfway to timeout
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Stream degraded: No frames for %ld seconds (timeout threshold: %.1f seconds)",
        elapsed, frame_timeout_seconds_);
    }
  }

  void publish_extrinsics_transform(const Extrinsics & extrinsics)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();

    tf_msg.header.frame_id = rgb_frame_id_;
    tf_msg.child_frame_id = depth_frame_id_;

    // 1. Load original rotation (Depth -> RGB)
    const float * r = extrinsics.rotation;
    tf2::Matrix3x3 m(
      r[0], r[3], r[6],
      r[1], r[4], r[7],
      r[2], r[5], r[8]);

    // 2. Load original translation (Depth -> RGB) in meters
    tf2::Vector3 t(
      extrinsics.translation[0] * 0.001,
      extrinsics.translation[1] * 0.001,
      extrinsics.translation[2] * 0.001
    );

    // 3. Calculate Inverse
    // Inverse Rotation (Transpose)
    tf2::Matrix3x3 m_inv = m.transpose();
    
    // Inverse Translation: - (R_inv * t)
    tf2::Vector3 t_inv = -(m_inv * t);

    // 4. Fill the message
    tf_msg.transform.translation.x = t_inv.x();
    tf_msg.transform.translation.y = t_inv.y();
    tf_msg.transform.translation.z = t_inv.z();

    tf2::Quaternion q;
    m_inv.getRotation(q);
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
      if (intr_out)
      {
        intr_out->width = static_cast<short>(cam_info.width);
        intr_out->height = static_cast<short>(cam_info.height);
        intr_out->fx = static_cast<float>(cam_info.k[0]);
        intr_out->fy = static_cast<float>(cam_info.k[4]);
        intr_out->cx = static_cast<float>(cam_info.k[2]);
        intr_out->cy = static_cast<float>(cam_info.k[5]);
      }
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
      try
      {
        self->handle_frame_pair(frame_dep, frame_rgb);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(self->get_logger(), "Exception in frame callback: %s", e.what());
        self->error_count_++;
      }
      catch (...)
      {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in frame callback");
        self->error_count_++;
      }
    }
  }

  void handle_frame_pair(cs::IFramePtr frame_dep, cs::IFramePtr frame_rgb)
  {
    // Update frame timestamp for watchdog
    last_frame_time_.store(std::chrono::steady_clock::now());
    frame_count_++;
    
    if (!frame_dep || frame_dep->empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received empty depth frame");
      error_count_++;
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
    if (!depth_data)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to get depth data from frame");
      error_count_++;
      return;
    }
    
    // Copy raw depth data first (for pointcloud generation)
    std::memcpy(depth_msg.data.data(), depth_data, depth_size);

    sensor_msgs::msg::CameraInfo depth_info = depth_info_msg_;
    depth_info.header.stamp = stamp;

    // Generate pointcloud BEFORE scaling depth image (uses raw SDK units)
    if (publish_pointcloud_ && cloud_pub_)
    {
      publish_pointcloud(depth_msg, frame_rgb);
    }

    // Now scale depth values for visualization in RVIZ
    // SDK depth values use depth_scale_sdk_ (e.g., 0.05 mm/unit)
    // ROS/RVIZ expects depth in millimeters (1.0 mm/unit)
    if (depth_scale_sdk_ != 1.0f)
    {
      uint16_t * dst = reinterpret_cast<uint16_t *>(depth_msg.data.data());
      const float scale_factor = depth_scale_sdk_;
      
      for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
        if (dst[i] != 0) {
          float depth_mm = dst[i] * scale_factor;
          dst[i] = static_cast<uint16_t>(std::min(depth_mm, 65535.0f));
        }
      }
    }

    depth_pub_->publish(depth_msg);
    depth_info_pub_->publish(depth_info);

    // Publish colorized depth visualization if enabled
    if (publish_depth_viz_ && depth_viz_pub_)
    {
      sensor_msgs::msg::Image depth_viz_msg;
      depth_viz_msg.header = depth_msg.header;
      depth_viz_msg.height = depth_msg.height;
      depth_viz_msg.width = depth_msg.width;
      depth_viz_msg.encoding = sensor_msgs::image_encodings::RGB8;
      depth_viz_msg.is_bigendian = false;
      depth_viz_msg.step = depth_msg.width * 3;
      depth_viz_msg.data.resize(depth_viz_msg.step * depth_viz_msg.height);

      const uint16_t * depth_ptr = reinterpret_cast<const uint16_t *>(depth_msg.data.data());
      uint8_t * viz_ptr = depth_viz_msg.data.data();

      // Find min/max for normalization
      uint16_t min_val = 65535, max_val = 0;
      for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
        if (depth_ptr[i] > 0) {
          min_val = std::min(min_val, depth_ptr[i]);
          max_val = std::max(max_val, depth_ptr[i]);
        }
      }

      // Colorize using jet colormap
      for (size_t i = 0; i < depth_msg.width * depth_msg.height; ++i) {
        uint16_t val = depth_ptr[i];
        if (val == 0) {
          viz_ptr[i * 3 + 0] = 0;
          viz_ptr[i * 3 + 1] = 0;
          viz_ptr[i * 3 + 2] = 0;
        } else {
          float normalized = (max_val > min_val) ? 
            static_cast<float>(val - min_val) / (max_val - min_val) : 0.0f;
          
          // Simple jet colormap
          float r, g, b;
          if (normalized < 0.25f) {
            r = 0.0f;
            g = normalized * 4.0f;
            b = 1.0f;
          } else if (normalized < 0.5f) {
            r = 0.0f;
            g = 1.0f;
            b = 1.0f - (normalized - 0.25f) * 4.0f;
          } else if (normalized < 0.75f) {
            r = (normalized - 0.5f) * 4.0f;
            g = 1.0f;
            b = 0.0f;
          } else {
            r = 1.0f;
            g = 1.0f - (normalized - 0.75f) * 4.0f;
            b = 0.0f;
          }
          
          viz_ptr[i * 3 + 0] = static_cast<uint8_t>(r * 255);
          viz_ptr[i * 3 + 1] = static_cast<uint8_t>(g * 255);
          viz_ptr[i * 3 + 2] = static_cast<uint8_t>(b * 255);
        }
      }

      depth_viz_pub_->publish(depth_viz_msg);
    }

    if (!frame_rgb || frame_rgb->empty())
    {
      // RGB frame is optional, just skip without error
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
    if (!rgb_data)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to get RGB data from frame");
      error_count_++;
      return;
    }
    
    std::memcpy(rgb_msg.data.data(), rgb_data, rgb_size);

    sensor_msgs::msg::CameraInfo rgb_info = rgb_info_msg_;
    rgb_info.header.stamp = stamp;

    rgb_pub_->publish(rgb_msg);
    rgb_info_pub_->publish(rgb_info);
  }

  void publish_pointcloud(const sensor_msgs::msg::Image & depth_msg, cs::IFramePtr frame_rgb)
  {
    if (depth_msg.encoding != sensor_msgs::image_encodings::TYPE_16UC1) return;

    const bool has_rgb_frame = frame_rgb && !frame_rgb->empty();

    // Use SDK's Pointcloud class to generate points with proper color projection
    cs::Pointcloud pc;
    const uint16_t * depth_data = reinterpret_cast<const uint16_t *>(depth_msg.data.data());
    unsigned char * rgb_data = has_rgb_frame ? (unsigned char *)frame_rgb->getData() : nullptr;
    
    // Generate points using SDK - it handles all the complex projection internally
    // SDK expects scale in mm/unit and produces points in mm (verified by PLY output)
    pc.generatePoints(
      depth_data,
      depth_msg.width,
      depth_msg.height,
      depth_scale_sdk_,  // mm/unit -> SDK outputs points in mm
      &depth_intrinsics_,
      has_rgb_frame ? &rgb_intrinsics_ : nullptr,
      has_rgb_frame ? &extrinsics_ : nullptr,
      false  // removeInvalid = false, keep all points
    );

    // Create ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = depth_msg.header.stamp;
    cloud.header.frame_id = depth_frame_id_;
    cloud.height = 1;  // Unordered cloud
    cloud.width = pc.getVertices().size();
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    if (has_rgb_frame && !pc.getTexcoords().empty()) {
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    } else {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(cloud.width);

    // Copy points from SDK to ROS message
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_rgb;
    if (has_rgb_frame && !pc.getTexcoords().empty()) {
      iter_rgb = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(cloud, "rgb");
    }

    int colored_count = 0;
    const auto & points = pc.getVertices();
    const auto & textures = pc.getTexcoords();
    
    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      const cs::float3 & pt = points[i];
      // SDK outputs points in millimeters, convert to meters for ROS
      *iter_x = pt.x * 0.001f;
      *iter_y = pt.y * 0.001f;
      *iter_z = pt.z * 0.001f;

      if (iter_rgb && i < textures.size()) {
        const cs::float2 & tex = textures[i];
        // Texture coords are normalized [0,1], convert to pixel coords
        int tex_x = static_cast<int>(tex.u * (frame_rgb->getWidth() - 1));
        int tex_y = static_cast<int>(tex.v * (frame_rgb->getHeight() - 1));

        if (tex_x >= 0 && tex_x < frame_rgb->getWidth() &&
            tex_y >= 0 && tex_y < frame_rgb->getHeight()) {
          const size_t idx = (tex_y * frame_rgb->getWidth() + tex_x) * 3;
          uint8_t r = static_cast<uint8_t>(rgb_data[idx + 0]);
          uint8_t g = static_cast<uint8_t>(rgb_data[idx + 1]);
          uint8_t b = static_cast<uint8_t>(rgb_data[idx + 2]);

          // Pack RGB into float
          uint32_t rgb_packed = (static_cast<uint32_t>(r) << 16) |
                                (static_cast<uint32_t>(g) << 8) |
                                static_cast<uint32_t>(b);
          float rgb_float;
          std::memcpy(&rgb_float, &rgb_packed, sizeof(float));
          **iter_rgb = rgb_float;
          ++colored_count;
        } else {
          **iter_rgb = std::numeric_limits<float>::quiet_NaN();
        }
        ++(*iter_rgb);
      }
    }

    cloud_pub_->publish(cloud);
  }

  std::string serial_;
  std::string depth_profile_;
  std::string rgb_profile_;

  std::string depth_frame_id_;
  std::string rgb_frame_id_;
  std::string depth_info_url_;
  std::string rgb_info_url_;

  bool publish_extrinsics_tf_ = true;
  bool publish_depth_scale_param_ = true;

  bool depth_auto_exposure_ = false;
  double depth_exposure_us_ = 0.0;
  double depth_frame_time_us_ = 0.0;
  double depth_gain_ = 0.0;
  int trigger_mode_ = 0;
  int depth_rgb_match_threshold_ = 0;
  int depth_rgb_match_rgb_offset_ = 0;
  bool depth_rgb_match_force_rgb_after_depth_ = false;

  bool initial_reset_ = false;
  bool clear_frame_buffer_ = false;

  bool laser_enable_ = true;
  int laser_luminance_ = 100;

  bool ir_led_enable_ = false;
  int ir_led_luminance_ = 0;
  std::string rgb_led_mode_;

  int depth_range_min_mm_ = 0;
  int depth_range_max_mm_ = 0;
  int algorithm_contrast_ = 0;

  bool publish_pointcloud_ = false;
  bool publish_depth_viz_ = false;

  std::string qos_profile_;

  cs::ICameraPtr camera_;
  StreamInfo depth_stream_info_{};
  StreamInfo rgb_stream_info_{};

  float depth_scale_ = 1.0f;  // Depth scale in meters/unit (for ROS)
  float depth_scale_sdk_ = 0.1f;  // Depth scale in mm/unit (for SDK generatePoints)

  Intrinsics depth_intrinsics_{};
  Intrinsics rgb_intrinsics_{};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_viz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_;

  sensor_msgs::msg::CameraInfo depth_info_msg_;
  sensor_msgs::msg::CameraInfo rgb_info_msg_;

  Extrinsics extrinsics_{};
  bool has_extrinsics_ = false;

  CameraInfo last_camera_info_{};
  bool has_camera_info_ = false;

  // Stream health monitoring
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::atomic<std::chrono::steady_clock::time_point> last_frame_time_{std::chrono::steady_clock::time_point{}};
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<uint64_t> error_count_{0};
  double frame_timeout_seconds_ = 5.0;
  bool enable_watchdog_ = true;
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
