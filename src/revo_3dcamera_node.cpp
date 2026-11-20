#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
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
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/trigger.hpp"
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
  explicit RevopointCameraNode(const rclcpp::NodeOptions & options)
  : Node("revopoint_camera", options)
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

    // --- Additional SDK capabilities -------------------------------------
    // Tri-state ints: -1 leaves the device default untouched, 0 = off, 1 = on.
    // Every one of these is opt-in, so an unset parameter never issues a
    // property write and behaviour is unchanged from before they existed.
    hdr_mode_ = declare_parameter<std::string>("hdr_mode", "");
    hdr_scale_ = declare_parameter<std::vector<int64_t>>(
      "hdr_scale", std::vector<int64_t>{});
    depth_filter_level_ = declare_parameter<int>("depth_filter_level", -1);
    depth_roi_ = declare_parameter<std::vector<int64_t>>(
      "depth_roi", std::vector<int64_t>{});
    rgb_roi_ = declare_parameter<std::vector<int64_t>>(
      "rgb_roi", std::vector<int64_t>{});
    multiframe_fusion_ = declare_parameter<int>("multiframe_fusion", -1);
    fast_scan_mode_ = declare_parameter<int>("fast_scan_mode", -1);
    hardware_sync_ = declare_parameter<int>("hardware_sync", -1);
    trigger_out_enable_ = declare_parameter<int>("trigger_out_enable", -1);
    trigger_in_mode_ = declare_parameter<int>("trigger_in_mode", -1);
    indicator_light_mode_ = declare_parameter<std::string>("indicator_light_mode", "");
    max_frame_time_us_ = declare_parameter<double>("max_frame_time_us", -1.0);
    max_gain_ = declare_parameter<double>("max_gain", -1.0);

    rgb_exposure_us_ = declare_parameter<double>("rgb_exposure_us", -1.0);
    rgb_auto_exposure_ = declare_parameter<int>("rgb_auto_exposure", -1);
    rgb_auto_white_balance_ = declare_parameter<int>("rgb_auto_white_balance", -1);
    rgb_white_balance_ = declare_parameter<double>("rgb_white_balance", -1.0);
    rgb_gain_ = declare_parameter<double>("rgb_gain", -1.0);
    rgb_auto_focus_ = declare_parameter<int>("rgb_auto_focus", -1);
    rgb_focus_ = declare_parameter<double>("rgb_focus", -1.0);

    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);

    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", false);
    align_depth_ = declare_parameter<bool>("align_depth", true);
    publish_depth_viz_ = declare_parameter<bool>("publish_depth_viz", false);
    qos_profile_ = declare_parameter<std::string>("qos_profile", "sensor_data");
    qos_depth_ = declare_parameter<int>("qos_depth", 2);
    lazy_publish_ = declare_parameter<bool>("lazy_publish", true);
    pointcloud_async_ = declare_parameter<bool>("pointcloud_async", true);

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

    if (publish_diagnostics_)
    {
      diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)));
    }
    create_services();

    // Initialize frame monitoring
    frame_timeout_seconds_ = declare_parameter<double>("frame_timeout_seconds", 5.0);
    enable_watchdog_ = declare_parameter<bool>("enable_watchdog", true);
    
    // Validate timeout value
    if (frame_timeout_seconds_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid frame_timeout_seconds %.1f, using default 5.0", frame_timeout_seconds_);
      frame_timeout_seconds_ = 5.0;
    }
    
    // Pre-compute half timeout threshold to avoid repeated division
    half_timeout_seconds_ = frame_timeout_seconds_ / 2.0;
    
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
    // Stop the camera first so the SDK thread can no longer enqueue work, then
    // drain and join the point cloud worker.
    stop_camera();
    stop_cloud_worker();
  }

private:
  /**
   * @brief One staged frame pair awaiting point cloud construction.
   *
   * Buffers are reused across frames (assign() into existing capacity), so the
   * steady-state hand-off between the SDK thread and the worker allocates nothing.
   */
  struct CloudJob
  {
    std::vector<uint16_t> depth;
    std::vector<uint8_t> rgb;
    uint32_t width = 0;
    uint32_t height = 0;
    rclcpp::Time stamp;
  };

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
    // Depth of 1-2 is what you want for a camera: a late subscriber should get the
    // newest frame, not a backlog of stale multi-megabyte messages.
    const int depth = std::max(1, qos_depth_);

    if (qos_profile_ == "best_effort" || qos_profile_ == "sensor_data")
    {
      // Best effort: a dropped frame is cheaper than head-of-line blocking on a
      // ~30 MB point cloud. This is the right default for high-rate sensor data.
      return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
    }

    if (qos_profile_ != "reliable")
    {
      RCLCPP_WARN_ONCE(get_logger(),
        "Unknown QoS profile '%s', defaulting to 'reliable'. "
        "Valid options are: 'sensor_data', 'best_effort', 'reliable'",
        qos_profile_.c_str());
    }

    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
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
      configure_advanced();
      configure_rgb_controls();
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
    configure_advanced();
    configure_rgb_controls();
    update_camera_info();

    return true;
  }

  void stop_camera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
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

  void initialize_rgb_parameters()
  {
    // Pre-compute RGB/depth frame dimensions once at stream start.
    rgb_width_ = rgb_stream_info_.width;
    rgb_height_ = rgb_stream_info_.height;
    depth_width_ = depth_stream_info_.width;
    depth_height_ = depth_stream_info_.height;
  }

  /**
   * @brief Pre-compute every constant the point cloud kernel needs.
   *
   * Called after update_camera_info(), i.e. once intrinsics, extrinsics and the
   * depth scale are known. Doing this here keeps the per-frame kernel free of
   * divisions and of the intrinsics rescaling that cs::Pointcloud redoes on
   * every single frame.
   */
  void initialize_projection_parameters()
  {
    // cs::Pointcloud rescales the depth intrinsics to the actual stream size on
    // every frame; we do it once. Same for RGB, which the SDK did NOT do (it
    // indexed the frame with intrinsics-resolution coordinates).
    const float dsx = (depth_intrinsics_.width > 0)
      ? static_cast<float>(depth_width_) / depth_intrinsics_.width : 1.0f;
    const float dsy = (depth_intrinsics_.height > 0)
      ? static_cast<float>(depth_height_) / depth_intrinsics_.height : 1.0f;

    const float fx = depth_intrinsics_.fx * dsx;
    const float fy = depth_intrinsics_.fy * dsy;
    proj_cx_ = depth_intrinsics_.cx * dsx;
    proj_cy_ = depth_intrinsics_.cy * dsy;
    proj_inv_fx_ = (fx != 0.0f) ? 1.0f / fx : 0.0f;
    proj_inv_fy_ = (fy != 0.0f) ? 1.0f / fy : 0.0f;

    const float rsx = (rgb_intrinsics_.width > 0)
      ? static_cast<float>(rgb_width_) / rgb_intrinsics_.width : 1.0f;
    const float rsy = (rgb_intrinsics_.height > 0)
      ? static_cast<float>(rgb_height_) / rgb_intrinsics_.height : 1.0f;
    rgb_fx_ = rgb_intrinsics_.fx * rsx;
    rgb_fy_ = rgb_intrinsics_.fy * rsy;
    rgb_cx_ = rgb_intrinsics_.cx * rsx;
    rgb_cy_ = rgb_intrinsics_.cy * rsy;

    // Depth raw units -> millimetres (the SDK convention this kernel works in).
    depth_unit_to_mm_ = depth_scale_sdk_;

    // Raw depth unit -> millimetre lookup table for the published 16UC1 image.
    // Replaces a float multiply + clamp + branch per pixel with one L2-resident
    // table lookup (64 KB).
    depth_mm_lut_identity_ = (std::abs(depth_scale_sdk_ - 1.0f) < 1e-6f);
    if (!depth_mm_lut_identity_)
    {
      depth_mm_lut_.resize(65536);
      depth_mm_lut_[0] = 0;
      for (size_t i = 1; i < 65536; ++i)
      {
        const float mm = static_cast<float>(i) * depth_scale_sdk_;
        depth_mm_lut_[i] = static_cast<uint16_t>(std::min(mm, 65535.0f));
      }
    }
    else
    {
      depth_mm_lut_.clear();
    }

    // 256-entry jet colormap, so the visualisation is a table lookup per pixel
    // instead of four branches and three float multiplies.
    build_jet_lut();

    RCLCPP_INFO(get_logger(),
      "Point cloud kernel ready: depth %dx%d (fx=%.2f fy=%.2f cx=%.2f cy=%.2f), "
      "rgb %dx%d (fx=%.2f fy=%.2f cx=%.2f cy=%.2f), %.6f mm/unit",
      depth_width_, depth_height_, fx, fy, proj_cx_, proj_cy_,
      rgb_width_, rgb_height_, rgb_fx_, rgb_fy_, rgb_cx_, rgb_cy_,
      static_cast<double>(depth_scale_sdk_));
  }

  void build_jet_lut()
  {
    jet_lut_.resize(256 * 3);
    for (int i = 0; i < 256; ++i)
    {
      const float n = static_cast<float>(i) / 255.0f;
      float r, g, b;
      if (n < 0.25f)      { r = 0.0f;              g = n * 4.0f;                b = 1.0f; }
      else if (n < 0.5f)  { r = 0.0f;              g = 1.0f;                    b = 1.0f - (n - 0.25f) * 4.0f; }
      else if (n < 0.75f) { r = (n - 0.5f) * 4.0f; g = 1.0f;                    b = 0.0f; }
      else                { r = 1.0f;              g = 1.0f - (n - 0.75f) * 4.0f; b = 0.0f; }
      jet_lut_[i * 3 + 0] = static_cast<uint8_t>(r * 255.0f);
      jet_lut_[i * 3 + 1] = static_cast<uint8_t>(g * 255.0f);
      jet_lut_[i * 3 + 2] = static_cast<uint8_t>(b * 255.0f);
    }
  }

  // ---------------------------------------------------------------------------
  // Point cloud worker thread
  // ---------------------------------------------------------------------------

  void start_cloud_worker()
  {
    if (!publish_pointcloud_ || !pointcloud_async_ || cloud_thread_.joinable())
    {
      return;
    }
    cloud_stop_ = false;
    cloud_thread_ = std::thread([this]() { cloud_worker(); });
    RCLCPP_INFO(get_logger(), "Point cloud worker thread started (latest-frame-wins)");
  }

  void stop_cloud_worker()
  {
    if (!cloud_thread_.joinable())
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_stop_ = true;
    }
    cloud_cv_.notify_all();
    cloud_thread_.join();
  }

  void cloud_worker()
  {
    CloudJob job;
    for (;;)
    {
      {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_cv_.wait(lock, [this]() { return cloud_job_pending_ || cloud_stop_; });
        if (cloud_stop_)
        {
          return;
        }
        // Swap rather than copy: O(1), and it hands our now-unused buffers back
        // to the producer so steady state does zero heap allocation.
        std::swap(job, pending_job_);
        cloud_job_pending_ = false;
      }

      try
      {
        build_and_publish_cloud(job);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(get_logger(), "Exception in point cloud worker: %s", e.what());
        error_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  bool start_streams()
  {
    ERROR_CODE ret = camera_->startStream(depth_stream_info_, rgb_stream_info_, &RevopointCameraNode::frame_pair_callback, this);
    if (ret == SUCCESS)
    {
      initialize_rgb_parameters();
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

    initialize_rgb_parameters();

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

    if (align_depth_)
    {
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

    // Depends on intrinsics, extrinsics and depth scale, so it must run last.
    initialize_projection_parameters();
    start_cloud_worker();
  }

  void check_stream_health()
  {
    auto now = std::chrono::steady_clock::now();
    int64_t last_frame_time_ns = last_frame_time_ns_.load();

    // Measured frame rate over the interval since the previous tick. This is
    // counted in the SDK callback, so it is what the camera actually delivered.
    double measured_fps = 0.0;
    const uint64_t frames_now = frame_count_.load(std::memory_order_relaxed);
    if (last_diag_time_.time_since_epoch().count() != 0)
    {
      const double dt = std::chrono::duration<double>(now - last_diag_time_).count();
      if (dt > 0.0 && frames_now >= last_diag_frame_count_)
      {
        measured_fps = static_cast<double>(frames_now - last_diag_frame_count_) / dt;
      }
    }
    last_diag_time_ = now;
    last_diag_frame_count_ = frames_now;

    const double since_frame = (last_frame_time_ns == 0)
      ? 0.0
      : std::chrono::duration<double>(
          now - std::chrono::steady_clock::time_point(
            std::chrono::steady_clock::duration(last_frame_time_ns))).count();

    publish_diagnostics(measured_fps, since_frame);

    // Skip the warning path if we haven't received any frames yet (startup).
    if (last_frame_time_ns == 0)
    {
      return;
    }
    
    // Reconstruct time_point from stored nanoseconds
    auto last_frame_time = std::chrono::steady_clock::time_point(
      std::chrono::steady_clock::duration(last_frame_time_ns));
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_frame_time).count();
    
    if (static_cast<double>(elapsed) > frame_timeout_seconds_)
    {
      RCLCPP_ERROR(get_logger(), 
        "Stream timeout! No frames received for %ld seconds. Camera may have failed.", 
        elapsed);
      
      // Log frame statistics
      auto total_frames = frame_count_.load();
      auto error_count = error_count_.load();
      RCLCPP_ERROR(get_logger(),
        "Frame statistics - Total: %lu, Errors: %lu, Clouds dropped: %lu, "
        "Last frame: %ld seconds ago",
        total_frames, error_count,
        static_cast<unsigned long>(cloud_dropped_.load(std::memory_order_relaxed)),
        elapsed);
    }
    else if (static_cast<double>(elapsed) > half_timeout_seconds_)
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

  // ===========================================================================
  // Additional SDK capabilities
  //
  // Every setter here is opt-in and fails soft: an unsupported property logs a
  // warning and startup continues, matching how the pre-existing configure_*
  // methods behave. Not every Revopoint model implements every property.
  // ===========================================================================

  /// -1 means "leave the device default alone".
  static bool is_set(int v) { return v >= 0; }
  static bool is_set(double v) { return v >= 0.0; }

  bool set_ext(PROPERTY_TYPE_EXTENSION prop, const PropertyExtension & value, const char * what)
  {
    const ERROR_CODE ret = camera_->setPropertyExtension(prop, value);
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(),
        "Failed to set %s (ret=%d) - this model may not support it", what, ret);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Set %s", what);
    return true;
  }

  bool set_prop(STREAM_TYPE stream, PROPERTY_TYPE prop, float value, const char * what)
  {
    const ERROR_CODE ret = camera_->setProperty(stream, prop, value);
    if (ret != SUCCESS)
    {
      RCLCPP_WARN(get_logger(),
        "Failed to set %s = %.2f (ret=%d) - this model may not support it",
        what, static_cast<double>(value), ret);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Set %s = %.2f", what, static_cast<double>(value));
    return true;
  }

  /// Parse a 4-element [left, top, right, bottom] ROI given in percent (0-100).
  bool parse_roi(const std::vector<int64_t> & in, int & l, int & t, int & r, int & b,
    const char * what)
  {
    if (in.empty())
    {
      return false;
    }
    if (in.size() != 4)
    {
      RCLCPP_WARN(get_logger(),
        "%s must have 4 elements [left, top, right, bottom] in percent, got %zu - ignoring",
        what, in.size());
      return false;
    }
    for (const auto v : in)
    {
      if (v < 0 || v > 100)
      {
        RCLCPP_WARN(get_logger(),
          "%s values are percentages and must be 0-100, got %ld - ignoring", what,
          static_cast<long>(v));
        return false;
      }
    }
    l = static_cast<int>(in[0]); t = static_cast<int>(in[1]);
    r = static_cast<int>(in[2]); b = static_cast<int>(in[3]);
    if (l >= r || t >= b)
    {
      RCLCPP_WARN(get_logger(),
        "%s is empty (left >= right or top >= bottom) - ignoring", what);
      return false;
    }
    return true;
  }

  void configure_advanced()
  {
    // --- HDR ---------------------------------------------------------------
    if (!hdr_mode_.empty())
    {
      HDR_MODE mode = HDR_MODE_OFF;
      bool ok = true;
      if (hdr_mode_ == "off") { mode = HDR_MODE_OFF; }
      else if (hdr_mode_ == "high_reflect") { mode = HDR_MODE_HIGH_RELECT; }
      else if (hdr_mode_ == "low_reflect") { mode = HDR_MODE_LOW_RELECT; }
      else if (hdr_mode_ == "all_reflect") { mode = HDR_MODE_ALL_RELECT; }
      else
      {
        RCLCPP_WARN(get_logger(),
          "Unknown hdr_mode '%s'. Valid: off, high_reflect, low_reflect, all_reflect",
          hdr_mode_.c_str());
        ok = false;
      }
      if (ok)
      {
        PropertyExtension p;
        p.hdrMode = mode;
        set_ext(PROPERTY_EXT_HDR_MODE, p, ("HDR mode '" + hdr_mode_ + "'").c_str());
      }
    }

    // Auto-HDR tuning: [highReflectCount, highReflectScale, lowReflectCount, lowReflectScale]
    if (!hdr_scale_.empty())
    {
      if (hdr_scale_.size() != 4)
      {
        RCLCPP_WARN(get_logger(),
          "hdr_scale must have 4 elements [high_count, high_scale, low_count, low_scale], "
          "got %zu - ignoring", hdr_scale_.size());
      }
      else
      {
        PropertyExtension p;
        p.hdrScaleSetting.highReflectModeCount = static_cast<unsigned int>(hdr_scale_[0]);
        p.hdrScaleSetting.highReflectModeScale = static_cast<unsigned int>(hdr_scale_[1]);
        p.hdrScaleSetting.lowReflectModeCount = static_cast<unsigned int>(hdr_scale_[2]);
        p.hdrScaleSetting.lowReflectModeScale = static_cast<unsigned int>(hdr_scale_[3]);
        set_ext(PROPERTY_EXT_HDR_SCALE_SETTING, p, "HDR scale setting");
      }
    }

    // --- Depth filtering ---------------------------------------------------
    if (is_set(depth_filter_level_))
    {
      PropertyExtension p;
      p.filterSwitch = depth_filter_level_;
      set_ext(PROPERTY_EXT_FILTERING_SWITCH, p, "depth filter level");
    }

    // --- ROIs --------------------------------------------------------------
    // A depth ROI is a real throughput lever: it reduces what the sensor
    // produces, which shrinks every downstream stage including the point cloud.
    int l, t, r, b;
    if (parse_roi(depth_roi_, l, t, r, b, "depth_roi"))
    {
      PropertyExtension p;
      p.depthRoi.left = l; p.depthRoi.top = t;
      p.depthRoi.right = r; p.depthRoi.bottom = b;
      set_ext(PROPERTY_EXT_DEPTH_ROI, p, "depth ROI");
    }
    if (parse_roi(rgb_roi_, l, t, r, b, "rgb_roi"))
    {
      PropertyExtension p;
      p.rgbRoi.left = l; p.rgbRoi.top = t;
      p.rgbRoi.right = r; p.rgbRoi.bottom = b;
      set_ext(PROPERTY_EXT_RGB_ROI, p, "RGB ROI");
    }

    // --- Capture modes -----------------------------------------------------
    if (is_set(multiframe_fusion_))
    {
      PropertyExtension p;
      p.multiframeFusion = (multiframe_fusion_ != 0);
      set_ext(PROPERTY_EXT_MULTIFRAME_FUSION, p, "multiframe fusion");
    }
    if (is_set(fast_scan_mode_))
    {
      PropertyExtension p;
      p.bFastScanMode = (fast_scan_mode_ != 0);
      set_ext(PROPERTY_EXT_FAST_SCAN_MODE, p, "fast scan mode");
    }
    if (is_set(hardware_sync_))
    {
      PropertyExtension p;
      p.bHardSyncSwitch = (hardware_sync_ != 0);
      set_ext(PROPERTY_EXT_HARDSYNCSWITCH, p, "hardware sync");
    }
    if (is_set(trigger_out_enable_))
    {
      PropertyExtension p;
      p.triggerOutEnable = trigger_out_enable_;
      set_ext(PROPERTY_EXT_TRIGGER_OUT_MODE, p, "trigger out");
    }
    if (is_set(trigger_in_mode_))
    {
      PropertyExtension p;
      p.triggerInMode = static_cast<TRIGGER_MODE>(trigger_in_mode_);
      set_ext(PROPERTY_EXT_TRIGGER_IN_MODE, p, "trigger in mode");
    }

    // --- Exposure/gain ceiling used by auto-exposure ------------------------
    if (is_set(max_frame_time_us_) || is_set(max_gain_))
    {
      PropertyExtension p;
      // Vendor-recommended values are 15000 (frame time) and 48 (gain).
      p.maxFrameTimeGain.iMaxFrame =
        is_set(max_frame_time_us_) ? static_cast<int>(max_frame_time_us_) : 15000;
      p.maxFrameTimeGain.iMaxGain =
        is_set(max_gain_) ? static_cast<int>(max_gain_) : 48;
      set_ext(PRPOERTY_EXT_MAX_FRAMETIME_GAIN, p, "max frame time / gain");
    }

    // --- Indicator light ---------------------------------------------------
    if (!indicator_light_mode_.empty())
    {
      INDICATOR_LIGHT_MODE mode = LIGHTING_OFF;
      bool ok = true;
      if (indicator_light_mode_ == "off") { mode = LIGHTING_OFF; }
      else if (indicator_light_mode_ == "red") { mode = RED_LIGHT_CONSTANTLY_LIT; }
      else if (indicator_light_mode_ == "red_flashing") { mode = RED_LIGHT_FLASHING; }
      else if (indicator_light_mode_ == "green") { mode = GREEN_LIGHT_CONSTANTLY_LIT; }
      else
      {
        RCLCPP_WARN(get_logger(),
          "Unknown indicator_light_mode '%s'. Valid: off, red, red_flashing, green",
          indicator_light_mode_.c_str());
        ok = false;
      }
      if (ok)
      {
        PropertyExtension p;
        p.indicatorLightMode = mode;
        set_ext(PROPERTY_EXT_INDICATOR_LIGHT_MODE, p, "indicator light mode");
      }
    }
  }

  /// RGB sensor controls. The driver previously exposed depth exposure/gain only.
  void configure_rgb_controls()
  {
    if (is_set(rgb_auto_exposure_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE,
        rgb_auto_exposure_ != 0 ? 1.0f : 0.0f, "RGB auto exposure");
    }
    if (is_set(rgb_exposure_us_))
    {
      // Dedicated extension property: microseconds, unlike PROPERTY_EXPOSURE
      // whose units are platform dependent.
      PropertyExtension p;
      p.uiExposureTime = static_cast<unsigned int>(rgb_exposure_us_);
      set_ext(PROPERTY_EXT_EXPOSURE_TIME_RGB, p, "RGB exposure time");
    }
    if (is_set(rgb_gain_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_GAIN, static_cast<float>(rgb_gain_), "RGB gain");
    }
    if (is_set(rgb_auto_white_balance_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_WHITEBALANCE,
        rgb_auto_white_balance_ != 0 ? 1.0f : 0.0f, "RGB auto white balance");
    }
    if (is_set(rgb_white_balance_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_WHITEBALANCE,
        static_cast<float>(rgb_white_balance_), "RGB white balance");
    }
    if (is_set(rgb_auto_focus_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_FOCUS,
        rgb_auto_focus_ != 0 ? 1.0f : 0.0f, "RGB auto focus");
    }
    if (is_set(rgb_focus_))
    {
      set_prop(STREAM_TYPE_RGB, PROPERTY_FOCUS, static_cast<float>(rgb_focus_), "RGB focus");
    }
  }

  // ===========================================================================
  // Services
  // ===========================================================================

  void create_services()
  {
    using std_srvs::srv::Trigger;

    pause_srv_ = create_service<Trigger>("~/pause_stream",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!camera_) { res->success = false; res->message = "camera not connected"; return; }
        const ERROR_CODE ret = camera_->pauseStream(STREAM_TYPE_DEPTH);
        res->success = (ret == SUCCESS);
        res->message = res->success ? "depth stream paused"
                                    : "pauseStream failed (ret=" + std::to_string(ret) + ")";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    resume_srv_ = create_service<Trigger>("~/resume_stream",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!camera_) { res->success = false; res->message = "camera not connected"; return; }
        const ERROR_CODE ret = camera_->resumeStream(STREAM_TYPE_DEPTH);
        res->success = (ret == SUCCESS);
        res->message = res->success ? "depth stream resumed"
                                    : "resumeStream failed (ret=" + std::to_string(ret) + ")";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // Only meaningful with trigger_mode:=2 (software trigger).
    soft_trigger_srv_ = create_service<Trigger>("~/soft_trigger",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!camera_) { res->success = false; res->message = "camera not connected"; return; }
        const ERROR_CODE ret = camera_->softTrigger(1);
        res->success = (ret == SUCCESS);
        res->message = res->success
          ? "soft trigger sent"
          : "softTrigger failed (ret=" + std::to_string(ret) +
            "); is trigger_mode set to 2 (software)?";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    reset_stats_srv_ = create_service<Trigger>("~/reset_stats",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res)
      {
        frame_count_.store(0, std::memory_order_relaxed);
        error_count_.store(0, std::memory_order_relaxed);
        cloud_dropped_.store(0, std::memory_order_relaxed);
        last_diag_frame_count_ = 0;
        res->success = true;
        res->message = "frame/error/dropped counters reset";
      });
  }

  // ===========================================================================
  // Diagnostics
  // ===========================================================================

  /// Read a temperature property, returning false if the model lacks it.
  bool read_temperatures(double & cpu, double & ntc, double & sensor)
  {
    bool any = false;
    PropertyExtension p;
    if (camera_->getPropertyExtension(PROPERTY_EXT_CPU_TEMPERATURE, p) == SUCCESS)
    {
      cpu = static_cast<double>(p.cpuTempratrue);
      any = true;
    }
    PropertyExtension n;
    if (camera_->getPropertyExtension(PROPERTY_EXT_NTC_TEMPERATURE, n) == SUCCESS &&
      n.ntcTempratrue.ntcCnt > 0)
    {
      ntc = static_cast<double>(n.ntcTempratrue.ntcTemp[0]);
      any = true;
    }
    PropertyExtension t;
    if (camera_->getPropertyExtension(PROPERTY_EXT_SENSOR_TEMPERATURE, t) == SUCCESS &&
      t.sensorTempratrue.sensorCnt > 0.0f)
    {
      sensor = static_cast<double>(t.sensorTempratrue.sensorTemp[0]);
      any = true;
    }
    return any;
  }

  static void add_kv(diagnostic_msgs::msg::DiagnosticStatus & st,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  }

  /**
   * @brief Publish measured frame rate, error counters and device temperatures.
   *
   * The measured FPS here is the number worth watching: it is counted in the
   * SDK callback, so it reflects what the camera actually delivered rather than
   * what any single topic published.
   */
  void publish_diagnostics(double measured_fps, double seconds_since_frame)
  {
    if (!diagnostics_pub_)
    {
      return;
    }

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = std::string(get_name()) + ": stream";
    st.hardware_id = has_camera_info_ ? last_camera_info_.serial : "unknown";

    const uint64_t errors = error_count_.load(std::memory_order_relaxed);
    const uint64_t dropped = cloud_dropped_.load(std::memory_order_relaxed);

    if (last_frame_time_ns_.load(std::memory_order_relaxed) == 0)
    {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "no frames received yet";
    }
    else if (seconds_since_frame > frame_timeout_seconds_)
    {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      st.message = "stream timeout";
    }
    else if (seconds_since_frame > half_timeout_seconds_)
    {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "stream degraded";
    }
    else
    {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = "streaming";
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f", measured_fps);
    add_kv(st, "measured_fps", buf);
    snprintf(buf, sizeof(buf), "%.1f", depth_stream_info_.fps);
    add_kv(st, "configured_depth_fps", buf);
    add_kv(st, "depth_resolution",
      std::to_string(depth_width_) + "x" + std::to_string(depth_height_));
    add_kv(st, "rgb_resolution",
      std::to_string(rgb_width_) + "x" + std::to_string(rgb_height_));
    add_kv(st, "frames_total",
      std::to_string(frame_count_.load(std::memory_order_relaxed)));
    add_kv(st, "errors", std::to_string(errors));
    add_kv(st, "pointclouds_dropped", std::to_string(dropped));
    snprintf(buf, sizeof(buf), "%.1f", seconds_since_frame);
    add_kv(st, "seconds_since_last_frame", buf);

    double cpu = 0.0, ntc = 0.0, sensor = 0.0;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_ && read_temperatures(cpu, ntc, sensor))
      {
        snprintf(buf, sizeof(buf), "%.1f", cpu);   add_kv(st, "cpu_temp_c", buf);
        snprintf(buf, sizeof(buf), "%.1f", ntc);   add_kv(st, "ntc_temp_c", buf);
        snprintf(buf, sizeof(buf), "%.1f", sensor); add_kv(st, "sensor_temp_c", buf);
      }
    }

    auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    msg->header.stamp = this->get_clock()->now();
    msg->status.push_back(st);
    diagnostics_pub_->publish(std::move(msg));
  }

  static void frame_pair_callback(cs::IFramePtr frame_dep, cs::IFramePtr frame_rgb, void * user_data)
  {
    auto * self = static_cast<RevopointCameraNode *>(user_data);
    if (!self)
    {
      return;
    }
    try
    {
      self->handle_frame_pair(frame_dep, frame_rgb);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(self->get_logger(), "Exception in frame callback: %s", e.what());
      self->error_count_.fetch_add(1, std::memory_order_relaxed);
    }
    catch (...)
    {
      RCLCPP_ERROR(self->get_logger(), "Unknown exception in frame callback");
      self->error_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  /**
   * @brief Runs on the SDK's own capture thread.
   *
   * Everything here must stay cheap: the SDK does not buffer, so any time spent
   * in this callback is time the driver is not draining USB, which shows up
   * directly as lost frames. Image publishing is memcpy-class work and stays
   * inline; point cloud construction is handed to a worker thread.
   */
  void handle_frame_pair(cs::IFramePtr frame_dep, cs::IFramePtr frame_rgb)
  {
    // 1. Health monitoring first, so the watchdog sees liveness even when every
    //    consumer has unsubscribed and we skip the work below.
    const auto now_steady = std::chrono::steady_clock::now();
    last_frame_time_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_steady.time_since_epoch()).count(),
      std::memory_order_relaxed);
    frame_count_.fetch_add(1, std::memory_order_relaxed);

    // 2. A single timestamp shared by every message derived from this pair.
    const rclcpp::Time stamp = this->get_clock()->now();

    // 3. Lazy publishing: with no subscribers there is nothing to compute. This
    //    is what keeps an idle driver from burning a core on point clouds.
    const bool want_rgb = !lazy_publish_ ||
      rgb_pub_->get_subscription_count() > 0 ||
      rgb_info_pub_->get_subscription_count() > 0;
    const bool want_depth_img = !lazy_publish_ ||
      depth_pub_->get_subscription_count() > 0 ||
      depth_info_pub_->get_subscription_count() > 0;
    const bool want_cloud = cloud_pub_ &&
      (!lazy_publish_ || cloud_pub_->get_subscription_count() > 0);
    const bool want_viz = depth_viz_pub_ &&
      (!lazy_publish_ || depth_viz_pub_->get_subscription_count() > 0);

    const bool rgb_valid = frame_rgb && !frame_rgb->empty();
    const char * rgb_src = rgb_valid ? frame_rgb->getData() : nullptr;

    // --- RGB FIRST, FOR MOVEIT ---
    // MoveIt's hand-eye plugin uses a message_filters synchroniser across the
    // RGB image and its camera_info; publishing them back-to-back before any
    // heavy work keeps that pair from being dropped.
    if (want_rgb && rgb_valid && rgb_src)
    {
      auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
      rgb_msg->header.stamp = stamp;
      rgb_msg->header.frame_id = rgb_frame_id_;
      rgb_msg->height = frame_rgb->getHeight();
      rgb_msg->width = frame_rgb->getWidth();
      rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg->is_bigendian = false;
      rgb_msg->step = static_cast<uint32_t>(rgb_msg->width * 3);

      const size_t rgb_size = static_cast<size_t>(rgb_msg->step) * rgb_msg->height;
      // assign() allocates and copies in one pass; resize() would zero-fill
      // ~5.7 MB first and then have it immediately overwritten.
      const uint8_t * src = reinterpret_cast<const uint8_t *>(rgb_src);
      rgb_msg->data.assign(src, src + rgb_size);

      auto rgb_info = std::make_unique<sensor_msgs::msg::CameraInfo>(rgb_info_msg_);
      rgb_info->header.stamp = stamp;
      if (rgb_info->d.empty())
      {
        rgb_info->d = {0.0, 0.0, 0.0, 0.0, 0.0};
      }

      rgb_pub_->publish(std::move(rgb_msg));
      rgb_info_pub_->publish(std::move(rgb_info));
    }

    // --- DEPTH ---
    if (!frame_dep || frame_dep->empty())
    {
      return;
    }
    if (!want_depth_img && !want_cloud && !want_viz)
    {
      return;
    }

    const char * depth_src = frame_dep->getData(FRAME_DATA_FORMAT_Z16);
    if (!depth_src)
    {
      return;
    }

    const uint32_t dw = static_cast<uint32_t>(frame_dep->getWidth());
    const uint32_t dh = static_cast<uint32_t>(frame_dep->getHeight());
    const size_t npix = static_cast<size_t>(dw) * dh;
    const uint16_t * raw = reinterpret_cast<const uint16_t *>(depth_src);

    // 4. Point cloud is built from RAW depth units, so hand it off BEFORE the
    //    millimetre rescale below. Getting this order wrong scales the cloud by
    //    depth_scale twice.
    if (want_cloud)
    {
      // Only hand RGB to the kernel when the frame matches the geometry the
      // projection constants were built for; otherwise fall back to XYZ-only
      // rather than indexing the colour buffer with mismatched dimensions.
      const bool rgb_usable = rgb_src && align_depth_ && rgb_valid &&
        frame_rgb->getWidth() == rgb_width_ &&
        frame_rgb->getHeight() == rgb_height_;
      enqueue_cloud_job(raw, dw, dh,
        rgb_usable ? reinterpret_cast<const uint8_t *>(rgb_src) : nullptr, stamp);
    }

    if (!want_depth_img && !want_viz)
    {
      return;
    }

    // 5. Raw units -> millimetres, fused into the copy. One pass over the image
    //    via a lookup table instead of copy-then-rescale with a float multiply
    //    and a branch per pixel.
    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = stamp;
    depth_msg->header.frame_id = depth_frame_id_;
    depth_msg->height = dh;
    depth_msg->width = dw;
    depth_msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg->is_bigendian = false;
    depth_msg->step = static_cast<uint32_t>(dw * sizeof(uint16_t));

    if (depth_mm_lut_identity_ || depth_mm_lut_.empty())
    {
      const uint8_t * src = reinterpret_cast<const uint8_t *>(depth_src);
      depth_msg->data.assign(src, src + npix * sizeof(uint16_t));
    }
    else
    {
      depth_msg->data.resize(npix * sizeof(uint16_t));
      uint16_t * dst = reinterpret_cast<uint16_t *>(depth_msg->data.data());
      const uint16_t * lut = depth_mm_lut_.data();
      for (size_t i = 0; i < npix; ++i)
      {
        dst[i] = lut[raw[i]];
      }
    }

    if (want_viz)
    {
      publish_depth_viz(*depth_msg);
    }

    if (want_depth_img)
    {
      auto depth_info = std::make_unique<sensor_msgs::msg::CameraInfo>(depth_info_msg_);
      depth_info->header.stamp = stamp;
      depth_pub_->publish(std::move(depth_msg));
      depth_info_pub_->publish(std::move(depth_info));
    }
  }

  /**
   * @brief Stage a frame pair for the cloud worker, dropping the previous one.
   *
   * Latest-frame-wins: if the worker is still busy we overwrite the pending job
   * rather than queueing, so a slow consumer can never build a backlog or stall
   * the SDK capture thread.
   */
  void enqueue_cloud_job(
    const uint16_t * depth, uint32_t width, uint32_t height,
    const uint8_t * rgb, const rclcpp::Time & stamp)
  {
    const size_t npix = static_cast<size_t>(width) * height;
    const size_t rgb_bytes = static_cast<size_t>(rgb_width_) * rgb_height_ * 3;

    if (!pointcloud_async_)
    {
      // Synchronous fallback: build inline on the SDK thread.
      sync_job_.depth.assign(depth, depth + npix);
      sync_job_.rgb.clear();
      if (rgb && rgb_bytes > 0)
      {
        sync_job_.rgb.assign(rgb, rgb + rgb_bytes);
      }
      sync_job_.width = width;
      sync_job_.height = height;
      sync_job_.stamp = stamp;
      build_and_publish_cloud(sync_job_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (cloud_job_pending_)
      {
        cloud_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
      // assign() reuses the existing capacity, so after the first frame this is
      // a pure memcpy with no allocation.
      pending_job_.depth.assign(depth, depth + npix);
      pending_job_.rgb.clear();
      if (rgb && rgb_bytes > 0)
      {
        pending_job_.rgb.assign(rgb, rgb + rgb_bytes);
      }
      pending_job_.width = width;
      pending_job_.height = height;
      pending_job_.stamp = stamp;
      cloud_job_pending_ = true;
    }
    cloud_cv_.notify_one();
  }

  static void add_point_field(
    sensor_msgs::msg::PointCloud2 & cloud, const char * name, uint32_t offset)
  {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  }

  /**
   * @brief Fused depth -> XYZ(+RGB) projection, written straight into the
   *        PointCloud2 payload.
   *
   * This replaces cs::Pointcloud::generatePoints(), which for a 1600x1200 depth
   * map did roughly this per frame:
   *   - two 23 MB malloc/free
   *   - a projection pass
   *   - calculateNormals(): two more passes, traversed column-major over a
   *     row-major array (a ~19 KB stride per access, so effectively one cache
   *     miss per pixel) producing normals this driver never uses
   *   - insertPoints(): 1.92M push_backs across three std::vectors
   * followed, back in this file, by a copy through PointCloud2Iterator.
   *
   * Here it is a single cache-friendly pass with no allocation, no normals, and
   * invalid points dropped instead of emitted as (0,0,0).
   */
  void build_and_publish_cloud(const CloudJob & job)
  {
    const uint32_t w = job.width;
    const uint32_t h = job.height;
    if (w == 0 || h == 0 || job.depth.size() < static_cast<size_t>(w) * h)
    {
      return;
    }

    const bool color = !job.rgb.empty() &&
      job.rgb.size() >= static_cast<size_t>(rgb_width_) * rgb_height_ * 3;

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp = job.stamp;
    cloud->header.frame_id = depth_frame_id_;
    cloud->height = 1;
    cloud->is_bigendian = false;
    // Invalid points are dropped rather than emitted as (0,0,0), so what we
    // publish contains no invalid entries.
    cloud->is_dense = true;

    const uint32_t point_step = color ? 16u : 12u;
    cloud->point_step = point_step;
    cloud->fields.clear();
    cloud->fields.reserve(color ? 4 : 3);
    add_point_field(*cloud, "x", 0);
    add_point_field(*cloud, "y", 4);
    add_point_field(*cloud, "z", 8);
    if (color)
    {
      add_point_field(*cloud, "rgb", 12);
    }

    // Size for the worst case, fill, then shrink. Shrinking a vector never
    // reallocates, so this costs one allocation instead of growth reallocations.
    cloud->data.resize(static_cast<size_t>(w) * h * point_step);

    const uint16_t * depth = job.depth.data();
    const uint8_t * rgb = color ? job.rgb.data() : nullptr;
    uint8_t * out = cloud->data.data();

    // Hoist every invariant out of the loop.
    const float unit_to_mm = depth_unit_to_mm_;
    const float inv_fx = proj_inv_fx_;
    const float inv_fy = proj_inv_fy_;
    const float cx = proj_cx_;
    const float cy = proj_cy_;
    const float r0 = extrinsics_.rotation[0], r1 = extrinsics_.rotation[1], r2 = extrinsics_.rotation[2];
    const float r3 = extrinsics_.rotation[3], r4 = extrinsics_.rotation[4], r5 = extrinsics_.rotation[5];
    const float r6 = extrinsics_.rotation[6], r7 = extrinsics_.rotation[7], r8 = extrinsics_.rotation[8];
    const float t0 = extrinsics_.translation[0];
    const float t1 = extrinsics_.translation[1];
    const float t2 = extrinsics_.translation[2];
    const float rfx = rgb_fx_, rfy = rgb_fy_, rcx = rgb_cx_, rcy = rgb_cy_;
    const int rw = rgb_width_;
    const int rh = rgb_height_;

    uint32_t count = 0;
    size_t idx = 0;

    for (uint32_t v = 0; v < h; ++v)
    {
      const float vy = (static_cast<float>(v) - cy) * inv_fy;
      for (uint32_t u = 0; u < w; ++u, ++idx)
      {
        const uint16_t raw = depth[idx];
        if (raw == 0)
        {
          continue;
        }

        // Millimetres, matching the SDK's internal convention (extrinsics
        // translation is in mm, so the RGB projection must happen in mm).
        const float z = static_cast<float>(raw) * unit_to_mm;
        const float x = (static_cast<float>(u) - cx) * inv_fx * z;
        const float y = vy * z;

        float * pt = reinterpret_cast<float *>(out);

        if (color)
        {
          // p_rgb = R * (p_depth + T), matching cs::Pointcloud::insertPoints.
          const float tx = x + t0;
          const float ty = y + t1;
          const float tz = z + t2;
          const float px = tx * r0 + ty * r1 + tz * r2;
          const float py = tx * r3 + ty * r4 + tz * r5;
          const float pz = tx * r6 + ty * r7 + tz * r8;
          if (pz <= 0.0f)
          {
            continue;
          }
          const float inv_pz = 1.0f / pz;
          const int fu = static_cast<int>(rfx * px * inv_pz + rcx);
          const int fv = static_cast<int>(rfy * py * inv_pz + rcy);
          if (fu < 0 || fu >= rw || fv < 0 || fv >= rh)
          {
            continue;
          }
          const size_t c = (static_cast<size_t>(fv) * rw + fu) * 3;
          const uint32_t packed =
            (static_cast<uint32_t>(rgb[c + 0]) << 16) |   // R
            (static_cast<uint32_t>(rgb[c + 1]) << 8) |    // G
            static_cast<uint32_t>(rgb[c + 2]);            // B
          std::memcpy(pt + 3, &packed, sizeof(uint32_t));
        }

        // SDK works in mm; ROS wants metres.
        pt[0] = x * 0.001f;
        pt[1] = y * 0.001f;
        pt[2] = z * 0.001f;

        out += point_step;
        ++count;
      }
    }

    cloud->width = count;
    cloud->row_step = count * point_step;
    cloud->data.resize(static_cast<size_t>(count) * point_step);

    cloud_pub_->publish(std::move(cloud));
  }

  void publish_depth_viz(const sensor_msgs::msg::Image & depth_msg)
  {
    auto viz = std::make_unique<sensor_msgs::msg::Image>();
    viz->header = depth_msg.header;
    viz->height = depth_msg.height;
    viz->width = depth_msg.width;
    viz->encoding = sensor_msgs::image_encodings::RGB8;
    viz->is_bigendian = false;
    viz->step = depth_msg.width * 3;
    viz->data.resize(static_cast<size_t>(viz->step) * viz->height);

    const size_t npix = static_cast<size_t>(depth_msg.width) * depth_msg.height;
    const uint16_t * src = reinterpret_cast<const uint16_t *>(depth_msg.data.data());
    uint8_t * dst = viz->data.data();

    uint16_t min_val = 65535;
    uint16_t max_val = 0;
    for (size_t i = 0; i < npix; ++i)
    {
      const uint16_t v = src[i];
      if (v > 0)
      {
        min_val = std::min(min_val, v);
        max_val = std::max(max_val, v);
      }
    }

    // Scale into the 256-entry jet table instead of recomputing the colormap
    // branches and float multiplies per pixel.
    const float span = (max_val > min_val)
      ? 255.0f / static_cast<float>(max_val - min_val) : 0.0f;
    const uint8_t * lut = jet_lut_.data();

    for (size_t i = 0; i < npix; ++i)
    {
      const uint16_t v = src[i];
      if (v == 0)
      {
        dst[i * 3 + 0] = 0;
        dst[i * 3 + 1] = 0;
        dst[i * 3 + 2] = 0;
        continue;
      }
      const int bin = static_cast<int>(static_cast<float>(v - min_val) * span);
      const size_t c = static_cast<size_t>(std::min(std::max(bin, 0), 255)) * 3;
      dst[i * 3 + 0] = lut[c + 0];
      dst[i * 3 + 1] = lut[c + 1];
      dst[i * 3 + 2] = lut[c + 2];
    }

    depth_viz_pub_->publish(std::move(viz));
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
  bool align_depth_ = true;
  bool publish_depth_viz_ = false;

  std::string qos_profile_;

  cs::ICameraPtr camera_;
  StreamInfo depth_stream_info_{};
  StreamInfo rgb_stream_info_{};

  float depth_scale_ = 1.0f;  // Depth scale in meters/unit (for ROS)
  float depth_scale_sdk_ = 0.1f;  // Depth scale in mm/unit (for SDK generatePoints)

  // Stream geometry, cached once at stream start.
  int rgb_width_ = 0;
  int rgb_height_ = 0;
  int depth_width_ = 0;
  int depth_height_ = 0;

  // Pre-computed point cloud projection constants (see
  // initialize_projection_parameters). Reciprocals are stored so the per-pixel
  // kernel contains no divisions.
  float proj_inv_fx_ = 0.0f;
  float proj_inv_fy_ = 0.0f;
  float proj_cx_ = 0.0f;
  float proj_cy_ = 0.0f;
  float rgb_fx_ = 0.0f;
  float rgb_fy_ = 0.0f;
  float rgb_cx_ = 0.0f;
  float rgb_cy_ = 0.0f;
  float depth_unit_to_mm_ = 0.1f;

  // Raw depth unit -> millimetre table for the published 16UC1 image.
  std::vector<uint16_t> depth_mm_lut_;
  bool depth_mm_lut_identity_ = false;
  // 256-entry jet colormap for the optional depth visualisation.
  std::vector<uint8_t> jet_lut_;

  // Point cloud worker: the SDK capture thread stages a job, this thread builds
  // and publishes the cloud. Latest-frame-wins, never blocks the producer.
  std::thread cloud_thread_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_cv_;
  CloudJob pending_job_;
  CloudJob sync_job_;
  bool cloud_job_pending_ = false;
  bool cloud_stop_ = false;
  std::atomic<uint64_t> cloud_dropped_{0};

  int qos_depth_ = 2;
  bool lazy_publish_ = true;
  bool pointcloud_async_ = true;

  // Additional SDK capabilities. -1 / "" / empty means "leave device default".
  std::string hdr_mode_;
  std::vector<int64_t> hdr_scale_;
  int depth_filter_level_ = -1;
  std::vector<int64_t> depth_roi_;
  std::vector<int64_t> rgb_roi_;
  int multiframe_fusion_ = -1;
  int fast_scan_mode_ = -1;
  int hardware_sync_ = -1;
  int trigger_out_enable_ = -1;
  int trigger_in_mode_ = -1;
  std::string indicator_light_mode_;
  double max_frame_time_us_ = -1.0;
  double max_gain_ = -1.0;

  double rgb_exposure_us_ = -1.0;
  int rgb_auto_exposure_ = -1;
  int rgb_auto_white_balance_ = -1;
  double rgb_white_balance_ = -1.0;
  double rgb_gain_ = -1.0;
  int rgb_auto_focus_ = -1;
  double rgb_focus_ = -1.0;

  // Guards camera_ against concurrent use by service callbacks, the diagnostics
  // timer and shutdown. The frame callback never touches camera_.
  std::mutex camera_mutex_;

  bool publish_diagnostics_ = true;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr soft_trigger_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_stats_srv_;
  uint64_t last_diag_frame_count_ = 0;
  std::chrono::steady_clock::time_point last_diag_time_{};

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
  std::atomic<int64_t> last_frame_time_ns_{0};  // Nanoseconds from steady_clock epoch (unspecified)
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<uint64_t> error_count_{0};
  double frame_timeout_seconds_ = 5.0;
  double half_timeout_seconds_ = 2.5;  // Pre-computed for efficiency
  bool enable_watchdog_ = true;
};

}  // namespace revopoint_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // Lets any subscriber composed into this process receive the published
  // unique_ptr messages without a serialise/copy round trip.
  options.use_intra_process_comms(true);
  auto node = std::make_shared<revopoint_camera::RevopointCameraNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
