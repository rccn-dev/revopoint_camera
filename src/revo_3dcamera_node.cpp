#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <condition_variable>
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

    initial_reset_ = declare_parameter<bool>("initial_reset", false);

    laser_enable_ = declare_parameter<bool>("laser_enable", true);
    laser_luminance_ = declare_parameter<int>("laser_luminance", 100);

    ir_led_enable_ = declare_parameter<bool>("ir_led_enable", false);
    ir_led_luminance_ = declare_parameter<int>("ir_led_luminance", 0);
    rgb_led_mode_ = declare_parameter<std::string>("rgb_led_mode", "disable");

    depth_range_min_mm_ = declare_parameter<int>("depth_range_min_mm", 0);
    depth_range_max_mm_ = declare_parameter<int>("depth_range_max_mm", 0);
    algorithm_contrast_ = declare_parameter<int>("algorithm_contrast", 0);

    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", false);
    align_depth_ = declare_parameter<bool>("align_depth", true);
    publish_depth_viz_ = declare_parameter<bool>("publish_depth_viz", false);
    qos_profile_ = declare_parameter<std::string>("qos_profile", "sensor_data");

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

    // Start dedicated publisher threads (decouples SDK callbacks from ROS publishing)
    start_publisher_threads();
    
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
    // Stop publisher threads before disconnecting camera
    stop_publisher_threads();
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
      return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    }
    else if (qos_profile_ == "best_effort" || qos_profile_ == "sensor_data")
    {
      // Best effort QoS (SensorDataQoS) — recommended for image streams
      return rclcpp::SensorDataQoS();
    }
    else
    {
      RCLCPP_WARN_ONCE(get_logger(), 
        "Unknown QoS profile '%s', defaulting to 'sensor_data'. "
        "Valid options are: 'reliable', 'best_effort', 'sensor_data'", 
        qos_profile_.c_str());
      return rclcpp::SensorDataQoS();
    }
  }

  /**
   * @brief Perform a clean camera restart via the C++ SDK restart() API.
   *
   * The Revopoint camera firmware retains internal state (frame rate, resolution
   * settings) across stop/start and even disconnect/reconnect cycles.  A full
   * restart is the only reliable way to clear that state and obtain predictable
   * frame rates on the next connection.
   *
   * Sequence: restart() → disconnect() → wait → re-discover → reconnect.
   */
  bool restart_camera_clean()
  {
    if (!camera_) {
      RCLCPP_WARN(get_logger(), "restart_camera_clean: no camera object");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Restarting camera for clean state...");
    ERROR_CODE ret = camera_->restart();
    if (ret != SUCCESS) {
      RCLCPP_WARN(get_logger(), "camera->restart() failed (ret=%d), attempting disconnect anyway", ret);
    }

    camera_->disconnect();
    camera_.reset();

    // Camera needs time to reboot firmware (empirically 5-10s over Ethernet)
    RCLCPP_INFO(get_logger(), "Waiting 8 seconds for camera firmware reboot...");
    std::this_thread::sleep_for(8s);

    // Re-discover and reconnect
    cs::ISystemPtr system = cs::getSystemPtr();
    for (int attempt = 0; attempt < 10; ++attempt) {
      std::vector<CameraInfo> cameras;
      ret = system->queryCameras(cameras);
      if (ret != SUCCESS || cameras.empty()) {
        RCLCPP_INFO(get_logger(), "Waiting for camera to reappear (%d/10)...", attempt + 1);
        std::this_thread::sleep_for(2s);
        continue;
      }

      CameraInfo selected = cameras[0];
      if (!serial_.empty()) {
        for (const auto & info : cameras) {
          if (serial_ == info.serial) { selected = info; break; }
        }
      }

      camera_ = cs::getCameraPtr();
      ret = camera_->connect(selected);
      if (ret != SUCCESS) {
        RCLCPP_WARN(get_logger(), "Reconnect failed (ret=%d), retry %d/10", ret, attempt + 1);
        camera_.reset();
        std::this_thread::sleep_for(2s);
        continue;
      }

      // Verify camera is responsive
      CameraInfo check{};
      if (camera_->getInfo(check) != SUCCESS) {
        RCLCPP_WARN(get_logger(), "Camera connected but not responsive, retry %d/10", attempt + 1);
        camera_->disconnect();
        camera_.reset();
        std::this_thread::sleep_for(2s);
        continue;
      }

      // Wait for stream infos to become available
      if (!wait_for_stream_infos(10, 500ms)) {
        RCLCPP_WARN(get_logger(), "Stream infos not ready after reconnect, retry %d/10", attempt + 1);
        camera_->disconnect();
        camera_.reset();
        std::this_thread::sleep_for(2s);
        continue;
      }

      log_camera_info();
      RCLCPP_INFO(get_logger(), "Camera restarted and reconnected successfully");
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Failed to reconnect after restart (10 attempts)");
    return false;
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

  // reconnect_after_restart and restart_and_reconnect consolidated into
  // restart_camera_clean() above.

  /**
   * @brief Main camera initialization sequence.
   *
   * Key learnings from SDK benchmarking:
   *   - The camera firmware retains state between sessions. A full restart()
   *     + disconnect/reconnect is the ONLY way to get a clean, predictable
   *     frame rate.  Without it, fps degrades progressively.
   *   - RGB frames are ONLY delivered when depth is also streaming (hardware
   *     constraint — the structured-light projector clocks the RGB sensor).
   *   - RGB frame rate is internally matched to depth frame rate by the
   *     camera firmware.
   *   - Network camera discovery requires setSdkEnableNetworking(true) and
   *     setEnableNetworking(true) before queryCameras().
   *
   * Sequence:
   *   1. Enable SDK networking (for Ethernet cameras)
   *   2. Discover and connect to camera
   *   3. If initial_reset: restart firmware for clean state
   *   4. Wait for stream info availability
   *   5. Select depth + RGB streams
   *   6. Configure camera properties (exposure, laser, etc.)
   *   7. Start depth stream first, let it settle, then start RGB
   *   8. Read camera info (intrinsics, extrinsics, depth scale)
   */
  bool start_camera()
  {
    // Step 1: Enable networking for Ethernet cameras
    cs::setSdkEnableNetworking(true);
    cs::setEnableNetworking(true);
    RCLCPP_INFO(get_logger(), "SDK networking enabled for Ethernet camera discovery");

    // Step 2: Discover cameras with retry (network cameras take time to appear)
    cs::ISystemPtr system = cs::getSystemPtr();
    std::vector<CameraInfo> cameras;
    ERROR_CODE ret;

    RCLCPP_INFO(get_logger(), "Searching for cameras...");
    for (int i = 0; i < 20; ++i) {
      ret = system->queryCameras(cameras);
      if (ret == SUCCESS && !cameras.empty()) break;
      if (i < 19) std::this_thread::sleep_for(500ms);
    }

    if (cameras.empty()) {
      RCLCPP_ERROR(get_logger(), "No cameras found after 10 seconds (ret=%d)", ret);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Found %zu camera(s)", cameras.size());

    // Select camera by serial or use first available
    CameraInfo selected = cameras[0];
    if (!serial_.empty()) {
      bool found = false;
      for (const auto & info : cameras) {
        if (serial_ == info.serial) {
          selected = info;
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_WARN(get_logger(), "Serial '%s' not found, using first camera (%s)",
          serial_.c_str(), selected.serial);
      }
    }

    // Step 2b: Connect
    camera_ = cs::getCameraPtr();
    ret = camera_->connect(selected);
    if (ret != SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Camera connect failed (ret=%d)", ret);
      return false;
    }
    log_camera_info();

    // Step 3: Restart for clean state if requested
    // This is CRITICAL for predictable frame rates — the camera firmware retains
    // internal state from previous sessions that degrades performance.
    if (initial_reset_) {
      RCLCPP_INFO(get_logger(), "Initial reset requested — restarting camera for clean state");
      if (!restart_camera_clean()) {
        RCLCPP_ERROR(get_logger(), "Camera restart failed");
        return false;
      }
      // restart_camera_clean() leaves us connected with stream infos ready
    }

    // Step 4: Wait for stream infos
    if (!initial_reset_ && !wait_for_stream_infos(10, 500ms)) {
      // Stream infos not ready — try a restart as last resort
      RCLCPP_WARN(get_logger(), "Stream infos not ready, attempting restart...");
      if (!restart_camera_clean()) {
        RCLCPP_ERROR(get_logger(), "Failed to get stream infos after restart");
        return false;
      }
    }

    // Step 5: Select streams
    if (!select_streams()) {
      return false;
    }

    // Step 6: Configure camera properties BEFORE starting streams
    // (some properties like exposure/gain affect the stream pipeline)
    configure_laser();
    configure_leds();
    configure_depth_exposure();
    configure_depth_controls();

    // Step 7: Start streams (depth first, then RGB — RGB requires active depth)
    if (!start_streams()) {
      RCLCPP_ERROR(get_logger(), "Failed to start streams");
      return false;
    }

    // Step 8: Read camera calibration data
    update_camera_info();

    return true;
  }

  void stop_camera()
  {
    if (!camera_) return;

    // Stop streams in reverse order (RGB first since it depends on depth)
    RCLCPP_INFO(get_logger(), "Stopping streams...");
    camera_->stopStream(STREAM_TYPE_RGB);
    std::this_thread::sleep_for(100ms);
    camera_->stopStream(STREAM_TYPE_DEPTH);
    std::this_thread::sleep_for(200ms);

    camera_->disconnect();
    camera_.reset();
    RCLCPP_INFO(get_logger(), "Camera disconnected");
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
    // Pre-compute RGB frame dimensions and texture scale factors once at stream start
    rgb_width_ = rgb_stream_info_.width;
    rgb_height_ = rgb_stream_info_.height;
    tex_scale_x_ = static_cast<float>(rgb_width_ - 1);
    tex_scale_y_ = static_cast<float>(rgb_height_ - 1);
  }

  /**
   * @brief Start depth and RGB streams with proper sequencing.
   *
   * Camera hardware constraints (confirmed by benchmark):
   *   - RGB cannot produce frames without an active depth stream
   *   - RGB frame rate is internally matched to depth frame rate
   *   - Depth needs a brief warm-up period before RGB starts reliably
   */
  bool start_streams()
  {
    // Step 1: Start depth first — RGB depends on depth being active
    ERROR_CODE ret = camera_->startStream(STREAM_TYPE_DEPTH, depth_stream_info_,
      &RevopointCameraNode::depth_frame_callback, this);
    if (ret != SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Depth stream start failed (ret=%d)", ret);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Depth stream started (%dx%d, format=%d)",
      depth_stream_info_.width, depth_stream_info_.height, depth_stream_info_.format);

    // Step 2: Let depth stream settle before starting RGB.
    // The camera's structured-light projector needs to stabilize and begin
    // clocking the RGB sensor. Without this delay, RGB may start with 0 fps.
    RCLCPP_INFO(get_logger(), "Waiting 2s for depth stream to stabilize...");
    std::this_thread::sleep_for(2s);

    // Step 3: Start RGB (will be internally synced to depth rate by firmware)
    ret = camera_->startStream(STREAM_TYPE_RGB, rgb_stream_info_,
      &RevopointCameraNode::rgb_frame_callback, this);
    if (ret != SUCCESS) {
      RCLCPP_ERROR(get_logger(), "RGB stream start failed (ret=%d)", ret);
      camera_->stopStream(STREAM_TYPE_DEPTH);
      return false;
    }

    initialize_rgb_parameters();
    RCLCPP_INFO(get_logger(),
      "Streams started: Depth %dx%d + RGB %dx%d (RGB synced to depth rate)",
      depth_stream_info_.width, depth_stream_info_.height,
      rgb_stream_info_.width, rgb_stream_info_.height);

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

    // NOTE: Do NOT set PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM here.
    // That property is for the getPairedFrame() polling API only. When used with
    // independent callbacks it causes the SDK to hold frames for timestamp matching,
    // starving both streams. The camera firmware already syncs RGB to depth internally.
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
    int64_t last_frame_time_ns = last_frame_time_ns_.load();
    
    // Skip check if we haven't received any frames yet (during startup)
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
        "Frame statistics - Total: %lu, Errors: %lu, Last frame: %ld seconds ago",
        total_frames, error_count, elapsed);
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

  // ---------------------------------------------------------------------------
  // Independent stream callbacks
  // ---------------------------------------------------------------------------

  static void depth_frame_callback(cs::IFramePtr frame, void * user_data)
  {
    auto * self = static_cast<RevopointCameraNode *>(user_data);
    if (self)
    {
      try
      {
        self->handle_depth_frame(frame);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(self->get_logger(), "Exception in depth callback: %s", e.what());
        self->error_count_.fetch_add(1);
      }
      catch (...)
      {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in depth callback");
        self->error_count_.fetch_add(1);
      }
    }
  }

  static void rgb_frame_callback(cs::IFramePtr frame, void * user_data)
  {
    auto * self = static_cast<RevopointCameraNode *>(user_data);
    if (self)
    {
      try
      {
        self->handle_rgb_frame(frame);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(self->get_logger(), "Exception in RGB callback: %s", e.what());
        self->error_count_.fetch_add(1);
      }
      catch (...)
      {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in RGB callback");
        self->error_count_.fetch_add(1);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // RGB handler — runs at native RGB rate (e.g. 20 Hz)
  // Lightweight: just caches frame and signals publisher thread.
  // ---------------------------------------------------------------------------

  void handle_rgb_frame(cs::IFramePtr frame_rgb)
  {
    if (!frame_rgb || frame_rgb->empty()) {
      return;
    }

    // Update watchdog (either stream proves the camera is alive)
    update_watchdog();

    // Cache the latest RGB frame for pointcloud colorization
    if (publish_pointcloud_ && align_depth_)
    {
      std::lock_guard<std::mutex> lock(rgb_cache_mutex_);
      cached_rgb_frame_ = frame_rgb;
    }

    // Queue frame for publishing on dedicated thread (returns immediately
    // so the SDK callback thread is not blocked by ROS serialization/transport)
    {
      std::lock_guard<std::mutex> lock(rgb_pub_mutex_);
      pending_rgb_frame_ = frame_rgb;
    }
    rgb_pub_cv_.notify_one();
  }

  // ---------------------------------------------------------------------------
  // Depth handler — runs at native depth rate (e.g. 6 Hz)
  // Lightweight: just caches frame and signals publisher thread.
  // ---------------------------------------------------------------------------

  void handle_depth_frame(cs::IFramePtr frame_dep)
  {
    if (!frame_dep || frame_dep->empty()) {
      return;
    }

    // Update watchdog
    update_watchdog();

    // Queue frame for publishing on dedicated thread
    {
      std::lock_guard<std::mutex> lock(depth_pub_mutex_);
      pending_depth_frame_ = frame_dep;
    }
    depth_pub_cv_.notify_one();
  }

  void update_watchdog()
  {
    auto now_system = std::chrono::steady_clock::now();
    int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now_system.time_since_epoch()).count();
    last_frame_time_ns_.store(now_ns);
    frame_count_.fetch_add(1);
  }

  // ---------------------------------------------------------------------------
  // Publisher threads — run processing + ROS publishing off the SDK callback
  // thread so frame delivery is never blocked by DDS serialization/transport.
  // ---------------------------------------------------------------------------

  void start_publisher_threads()
  {
    running_.store(true);
    rgb_pub_thread_ = std::thread([this]{ rgb_publisher_loop(); });
    depth_pub_thread_ = std::thread([this]{ depth_publisher_loop(); });
  }

  void stop_publisher_threads()
  {
    running_.store(false);
    rgb_pub_cv_.notify_all();
    depth_pub_cv_.notify_all();
    if (rgb_pub_thread_.joinable()) rgb_pub_thread_.join();
    if (depth_pub_thread_.joinable()) depth_pub_thread_.join();
  }

  void rgb_publisher_loop()
  {
    while (running_.load()) {
      cs::IFramePtr frame;
      {
        std::unique_lock<std::mutex> lock(rgb_pub_mutex_);
        rgb_pub_cv_.wait(lock, [this]{ return pending_rgb_frame_ != nullptr || !running_.load(); });
        if (!running_.load()) break;
        frame = std::move(pending_rgb_frame_);
        pending_rgb_frame_.reset();
      }
      if (!frame) continue;
      try {
        process_and_publish_rgb(frame);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception in RGB publisher: %s", e.what());
        error_count_.fetch_add(1);
      }
    }
  }

  void depth_publisher_loop()
  {
    while (running_.load()) {
      cs::IFramePtr frame;
      {
        std::unique_lock<std::mutex> lock(depth_pub_mutex_);
        depth_pub_cv_.wait(lock, [this]{ return pending_depth_frame_ != nullptr || !running_.load(); });
        if (!running_.load()) break;
        frame = std::move(pending_depth_frame_);
        pending_depth_frame_.reset();
      }
      if (!frame) continue;
      try {
        process_and_publish_depth(frame);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth publisher: %s", e.what());
        error_count_.fetch_add(1);
      }
    }
  }

  void process_and_publish_rgb(cs::IFramePtr frame_rgb)
  {
    rclcpp::Time stamp = this->get_clock()->now();

    sensor_msgs::msg::Image rgb_msg;
    rgb_msg.header.stamp = stamp;
    rgb_msg.header.frame_id = rgb_frame_id_;
    rgb_msg.height = frame_rgb->getHeight();
    rgb_msg.width = frame_rgb->getWidth();
    rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_msg.step = static_cast<uint32_t>(rgb_msg.width * 3);

    size_t rgb_size = rgb_msg.step * rgb_msg.height;
    rgb_msg.data.resize(rgb_size);
    const char * rgb_data = frame_rgb->getData();
    if (!rgb_data) {
      return;
    }
    std::memcpy(rgb_msg.data.data(), rgb_data, rgb_size);

    // Prepare CameraInfo
    sensor_msgs::msg::CameraInfo rgb_info = rgb_info_msg_;
    rgb_info.header.stamp = stamp;

    // Ensure distortion vector is not empty (MoveIt requirement)
    if (rgb_info.d.empty()) {
      rgb_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    rgb_pub_->publish(rgb_msg);
    rgb_info_pub_->publish(rgb_info);
  }

  void process_and_publish_depth(cs::IFramePtr frame_dep)
  {
    rclcpp::Time stamp = this->get_clock()->now();

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = depth_frame_id_;
    depth_msg.height = frame_dep->getHeight();
    depth_msg.width = frame_dep->getWidth();
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg.step = static_cast<uint32_t>(depth_msg.width * sizeof(uint16_t));

    size_t depth_size = depth_msg.step * depth_msg.height;
    depth_msg.data.resize(depth_size);
    const char * depth_data = frame_dep->getData(FRAME_DATA_FORMAT_Z16);

    if (!depth_data) {
      return;
    }
    std::memcpy(depth_msg.data.data(), depth_data, depth_size);

    // Prepare Depth CameraInfo
    sensor_msgs::msg::CameraInfo depth_info = depth_info_msg_;
    depth_info.header.stamp = stamp;

    // Generate pointcloud (uses cached RGB frame if align_depth is on)
    if (publish_pointcloud_ && cloud_pub_) {
      cs::IFramePtr rgb_for_pc;
      if (align_depth_) {
        std::lock_guard<std::mutex> lock(rgb_cache_mutex_);
        rgb_for_pc = cached_rgb_frame_;  // may be null if no RGB yet
      }
      publish_pointcloud(depth_msg, rgb_for_pc);
    }

    // Apply depth scale conversion to mm
    if (depth_scale_sdk_ != 1.0f) {
      uint16_t * dst = reinterpret_cast<uint16_t *>(depth_msg.data.data());
      const float scale_factor = depth_scale_sdk_;
      const size_t num_pixels = depth_msg.width * depth_msg.height;
      for (size_t i = 0; i < num_pixels; ++i) {
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
        // Texture coords are normalized [0,1], convert to pixel coords using pre-computed scale
        const int tex_x = static_cast<int>(tex.u * tex_scale_x_);
        const int tex_y = static_cast<int>(tex.v * tex_scale_y_);

        if (tex_x >= 0 && tex_x < rgb_width_ &&
            tex_y >= 0 && tex_y < rgb_height_) {
          const size_t idx = (tex_y * rgb_width_ + tex_x) * 3;
          
          // Pack RGB into float (RGB order: R=idx+0, G=idx+1, B=idx+2)
          const uint32_t rgb_packed = (static_cast<uint32_t>(rgb_data[idx + 0]) << 16) |  // R
                                      (static_cast<uint32_t>(rgb_data[idx + 1]) << 8) |   // G
                                      static_cast<uint32_t>(rgb_data[idx + 2]);           // B
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

  bool initial_reset_ = false;

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

  // Pre-computed RGB stream parameters (set once at stream start)
  int rgb_width_ = 0;
  int rgb_height_ = 0;
  float tex_scale_x_ = 0.0f;
  float tex_scale_y_ = 0.0f;

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

  // RGB frame cache for pointcloud colorization (written by RGB callback, read by depth callback)
  std::mutex rgb_cache_mutex_;
  cs::IFramePtr cached_rgb_frame_;

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

  // Publisher threads — decouple SDK callbacks from ROS publishing
  std::atomic<bool> running_{false};

  std::mutex rgb_pub_mutex_;
  std::condition_variable rgb_pub_cv_;
  cs::IFramePtr pending_rgb_frame_;
  std::thread rgb_pub_thread_;

  std::mutex depth_pub_mutex_;
  std::condition_variable depth_pub_cv_;
  cs::IFramePtr pending_depth_frame_;
  std::thread depth_pub_thread_;
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
