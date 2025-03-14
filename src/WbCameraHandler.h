#ifndef WB_CAMERA_HANDLER_H
#define WB_CAMERA_HANDLER_H

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <sys/mman.h>
#include <unordered_map>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

namespace camera
{

// Shared timestamp structure for hardware sync
struct alignas(64) time_stamp
{
  int64_t high;
  int64_t low;
};

// Log level enum
enum class LogLevel
{
  DEBUG,
  INFO,
  WARN,
  ERROR
};

class WbCameraHandler
{
public:
  // Frame data structure to pass data back to the ROS node
  struct FrameData
  {
    const uint8_t *data;
    size_t size;
    int width;
    int height;
    int stride;
    uint64_t camera_timestamp_ns;
    uint64_t hw_timestamp_ns;
    bool hw_timestamp_valid;
  };

  // Callback for when a frame is ready
  using FrameCallback = std::function<void(const FrameData &)>;

  // Callback for log messages
  using LogCallback = std::function<void(LogLevel level, const std::string &message)>;

  WbCameraHandler();
  ~WbCameraHandler();

  bool
  initialize(const std::string &camera_id, uint32_t width, uint32_t height);

  bool
  start(FrameCallback callback);

  void
  stop();

  // Getters
  std::string
  getCameraName() const;

  bool
  isRunning() const
  {
    return running;
  }

  // Set log callback
  void
  setLogCallback(LogCallback callback)
  {
    log_callback = callback;
  }

  // Configuration methods
  void
  setExposure(bool auto_exposure, uint32_t exposure_time_us);

  bool
  setupHardwareSync(const std::string &timeshare_path);

  // Image manipulation
  static void
  flipRGB888Image(uint8_t *data, int width, int height, int step);

private:
  // LibCamera components
  libcamera::CameraManager camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
  libcamera::Stream *stream;
  std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::unique_ptr<libcamera::Request>> requests;

  // Buffer management
  struct buffer_info_t
  {
    void *data;
    size_t size;
  };
  std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info;
  std::mutex buffer_mutex;

  // Camera settings
  std::string camera_name;
  bool auto_exposure_enabled;
  uint32_t exposure_time_us;

  // Timestamp handling
  time_stamp *pointt;
  std::string timeshare_path;

  // State tracking
  std::atomic<bool> running {false};
  FrameCallback frame_callback;
  LogCallback log_callback;

  // Logging methods
  void
  logDebug(const std::string &message);
  void
  logInfo(const std::string &message);
  void
  logWarn(const std::string &message);
  void
  logError(const std::string &message);

  void
  requestComplete(libcamera::Request *request);

  bool
  configureCamera(uint32_t width, uint32_t height);
};
} // namespace camera

#endif // WB_CAMERA_HANDLER_H
