#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "WbCameraHandler.h"

namespace camera
{

class MinimalCameraNode : public rclcpp::Node
{
public:
  explicit MinimalCameraNode(const rclcpp::NodeOptions &options)
      : Node("camera", options),
        cim(this)
  {
    RCLCPP_INFO(get_logger(), "Starting MinimalCameraNode initialization...");

    // Parameters
    const uint32_t width = declare_parameter<int64_t>("width", 1280);
    const uint32_t height = declare_parameter<int64_t>("height", 720);
    const std::string camera_id = declare_parameter<std::string>("camera", "");
    const bool flip_image = declare_parameter<bool>("flip_image", true);
    const bool auto_exposure = declare_parameter<bool>("auto_exposure", false);
    const uint32_t exposure_time_us = declare_parameter<int64_t>("exposure_time_us", 90000);

    RCLCPP_INFO(get_logger(), "Parameters loaded - width: %u, height: %u, camera_id: '%s', flip: %s, auto_exposure: %s, exposure_time: %u μs",
                width, height, camera_id.c_str(), flip_image ? "true" : "false",
                auto_exposure ? "true" : "false", exposure_time_us);

    // Set up path to shared timestamp file
    const char *user_name = getlogin();
    if (!user_name) {
      RCLCPP_ERROR(get_logger(), "Failed to get username for timeshare path");
      throw std::runtime_error("Failed to get username");
    }
    std::string timeshare_path = "/home/" + std::string(user_name) + "/timeshare";
    RCLCPP_INFO(get_logger(), "Using timeshare path: %s", timeshare_path.c_str());

    // Create publishers
    pub_image = create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10);
    pub_ci = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

    RCLCPP_INFO(get_logger(), "Publishing to topics:");
    RCLCPP_INFO(get_logger(), "  - %s", pub_image->get_topic_name());
    RCLCPP_INFO(get_logger(), "  - %s", pub_ci->get_topic_name());

    RCLCPP_INFO(get_logger(), "Creating WbCameraHandler...");
    try {
      // Initialize camera handler
      camera_handler = std::make_unique<WbCameraHandler>();

      RCLCPP_INFO(get_logger(), "Initializing camera...");
      if (!camera_handler->initialize(camera_id, width, height)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize camera");
        throw std::runtime_error("Failed to initialize camera");
      }
      RCLCPP_INFO(get_logger(), "Camera initialized successfully");

      // Set exposure settings
      RCLCPP_INFO(get_logger(), "Setting camera exposure...");
      camera_handler->setExposure(auto_exposure, exposure_time_us);

      // Set up hardware synchronization
      RCLCPP_INFO(get_logger(), "Setting up hardware sync...");
      bool sync_result = camera_handler->setupHardwareSync(timeshare_path);
      RCLCPP_INFO(get_logger(), "Hardware sync setup %s", sync_result ? "successful" : "failed (will use system time)");

      // Set up camera info manager
      const std::string camera_name = camera_handler->getCameraName();
      RCLCPP_INFO(get_logger(), "Using camera name: %s", camera_name.c_str());
      cim.setCameraName(camera_name);

      const std::string &camera_info_url = declare_parameter<std::string>("camera_info_url", "");
      RCLCPP_INFO(get_logger(), "Loading camera calibration from URL: %s",
                  camera_info_url.empty() ? "(empty)" : camera_info_url.c_str());
      if (!cim.loadCameraInfo(camera_info_url)) {
        RCLCPP_WARN(get_logger(), "Failed to load camera calibration from URL");
      }

      // Set up the frame callback
      RCLCPP_INFO(get_logger(), "Setting up frame callback...");
      auto frame_callback = [this, flip_image](const WbCameraHandler::FrameData &frame_data) {
        handleFrame(frame_data, flip_image);
      };

      // Start the camera
      RCLCPP_INFO(get_logger(), "Starting camera...");
      if (!camera_handler->start(frame_callback)) {
        RCLCPP_ERROR(get_logger(), "Failed to start camera");
        throw std::runtime_error("Failed to start camera");
      }
      RCLCPP_INFO(get_logger(), "Camera started successfully");

      // Set up statistics timer
      RCLCPP_INFO(get_logger(), "Setting up statistics timer...");

      RCLCPP_INFO(get_logger(), "MinimalCameraNode initialization complete");
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception during camera initialization: %s", e.what());
      throw;
    }
    catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown exception during camera initialization");
      throw;
    }
  }

  ~MinimalCameraNode()
  {
    RCLCPP_INFO(get_logger(), "Shutting down MinimalCameraNode...");
    if (camera_handler) {
      RCLCPP_INFO(get_logger(), "Stopping camera...");
      camera_handler->stop();
      RCLCPP_INFO(get_logger(), "Camera stopped");
    }
    RCLCPP_INFO(get_logger(), "MinimalCameraNode shutdown complete");
  }

private:
  // Camera handler
  std::unique_ptr<WbCameraHandler> camera_handler;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;

  // Camera info manager
  camera_info_manager::CameraInfoManager cim;

  // Timer for periodic stats printing
  rclcpp::TimerBase::SharedPtr stats_timer;

  void
  handleFrame(const WbCameraHandler::FrameData &frame_data, bool flip_image)
  {
    // Log the first few frames for debugging
    static int frame_debug_count = 0;
    if (frame_debug_count < 5) {
      RCLCPP_INFO(get_logger(), "Received frame %d: %dx%d, HW timestamp: %ld (valid: %s), camera timestamp: %ld",
                  frame_debug_count, frame_data.width, frame_data.height,
                  frame_data.hw_timestamp_ns, frame_data.hw_timestamp_valid ? "yes" : "no",
                  frame_data.camera_timestamp_ns);
      frame_debug_count++;
    }

    // Check if there are any subscribers before processing
    bool has_raw_subscribers = pub_image->get_subscription_count() > 0;
    bool has_camera_info_subscribers = pub_ci->get_subscription_count() > 0;

    if (!has_raw_subscribers && !has_camera_info_subscribers) {
      // Periodically log when no subscribers
      static int no_sub_log_count = 0;
      if (no_sub_log_count++ % 100 == 0) {
        RCLCPP_INFO(get_logger(), "No subscribers to image or camera_info topics");
      }
      return;
    }

    // Prepare header and timestamp
    std_msgs::msg::Header hdr;
    hdr.frame_id = "camera";

    // Format the timestamp properly for ROS messages
    if (frame_data.hw_timestamp_valid) {
      // Convert to ROS time:
      // The LiDAR puts nanoseconds since Unix epoch in hw_timestamp_ns
      // This is exactly what ROS2 expects for its time representation
      hdr.stamp.sec = frame_data.hw_timestamp_ns / 1000000000;
      hdr.stamp.nanosec = frame_data.hw_timestamp_ns % 1000000000;
    }
    else if (frame_data.camera_timestamp_ns > 0) {
      // Fall back to camera timestamp if available
      hdr.stamp.sec = frame_data.camera_timestamp_ns / 1000000000;
      hdr.stamp.nanosec = frame_data.camera_timestamp_ns % 1000000000;
    }
    else {
      // Last resort is system time
      hdr.stamp = this->now();
    }

    try {
      // Publish raw image if there are subscribers
      if (has_raw_subscribers) {
        auto msg_img = std::make_unique<sensor_msgs::msg::Image>();
        msg_img->header = hdr;
        msg_img->width = frame_data.width;
        msg_img->height = frame_data.height;
        msg_img->step = frame_data.stride;
        msg_img->encoding = "rgb8";
        msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);

        msg_img->data.resize(frame_data.size);
        memcpy(msg_img->data.data(), frame_data.data, frame_data.size);

        if (flip_image) {
          WbCameraHandler::flipRGB888Image(msg_img->data.data(), frame_data.width,
                                           frame_data.height, frame_data.stride);
        }

        pub_image->publish(std::move(msg_img));
      }

      // Publish camera info if there are subscribers
      if (has_camera_info_subscribers) {
        sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
        ci.header = hdr;
        pub_ci->publish(ci);
      }
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception during frame publishing: %s", e.what());
    }
  }
};

} // namespace camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::MinimalCameraNode)

// #include <atomic>
// #include <chrono>
// #include <fcntl.h>
// #include <memory>
// #include <mutex>
// #include <string>
// #include <sys/mman.h>
// #include <sys/stat.h> // For stat() function
// #include <thread>
// #include <unistd.h>
// #include <vector>

// #include <camera_info_manager/camera_info_manager.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/msg/image.hpp>

// #include <libcamera/camera.h>
// #include <libcamera/camera_manager.h>
// #include <libcamera/control_ids.h>
// #include <libcamera/formats.h>
// #include <libcamera/framebuffer_allocator.h>
// #include <libcamera/property_ids.h>
// #include <libcamera/request.h>
// #include <libcamera/stream.h>

// namespace camera
// {

// // Shared timestamp structure for hardware sync
// struct alignas(64) time_stamp
// {
//   int64_t high;
//   int64_t low;
// };

// class MinimalCameraNode : public rclcpp::Node
// {
// public:
//   explicit MinimalCameraNode(const rclcpp::NodeOptions &options);
//   ~MinimalCameraNode();

// private:
//   // LibCamera components
//   libcamera::CameraManager camera_manager;
//   std::shared_ptr<libcamera::Camera> camera;
//   libcamera::Stream *stream;
//   std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
//   std::vector<std::unique_ptr<libcamera::Request>> requests;

//   // Buffer management
//   struct buffer_info_t
//   {
//     void *data;
//     size_t size;
//   };
//   std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info;
//   std::mutex buffer_mutex;

//   // Image flipping
//   bool flip_image = true;

//   // Publishers
//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
//   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;

//   // Camera info manager
//   camera_info_manager::CameraInfoManager cim;

//   // Timestamp handling
//   time_stamp *pointt = (time_stamp *)MAP_FAILED;
//   std::string timeshare_path;
//   std::atomic<bool> running {false};

//   // Exposure settings (configurable)
//   const bool AE_ENABLE = false;
//   const uint32_t EXPOSURE_TIME_US = 90000; // 90ms exposure in µs


//   struct TimestampStats
//   {
//     int64_t first_hw_timestamp = 0;   // First hardware timestamp seen
//     int64_t last_hw_timestamp = 0;    // Last hardware timestamp seen
//     int64_t frame_count = 0;          // Total frames processed
//     int64_t hw_timestamp_count = 0;   // Frames with hardware timestamps
//     double max_timestamp_jump = 0.0;  // Maximum jump between consecutive timestamps (ms)
//     double min_timestamp_jump = 1e9;  // Minimum jump between consecutive timestamps (ms)
//     double sum_timestamp_jumps = 0.0; // Sum of all jumps for averaging
//   } timestamp_stats;

//   // Timer for periodic stats printing
//   rclcpp::TimerBase::SharedPtr stats_timer;
//   void
//   printTimestampStats();


//   // Methods
//   void
//   requestComplete(libcamera::Request *request);
//   bool
//   setupTimeshare();
//   void
//   flipRGB888Image(uint8_t *data, int width, int height, int step);
// };

// // Constructor
// MinimalCameraNode::MinimalCameraNode(const rclcpp::NodeOptions &options)
//     : Node("camera", options),
//       cim(this)
// {
//   // Parameters
//   const uint32_t width = declare_parameter<int64_t>("width", 1280);
//   const uint32_t height = declare_parameter<int64_t>("height", 720);
//   const libcamera::Size size {width, height};

//   const std::string camera_id = declare_parameter<std::string>("camera", "");

//   // Set up path to shared timestamp file
//   const char *user_name = getlogin();
//   timeshare_path = "/home/" + std::string(user_name) + "/timeshare";
//   setupTimeshare();

//   // Create publishers
//   pub_image = create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10);
//   pub_ci = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

//   RCLCPP_INFO(get_logger(), "Publishing to topics:");
//   RCLCPP_INFO(get_logger(), "  - %s", pub_image->get_topic_name());
//   RCLCPP_INFO(get_logger(), "  - %s", pub_ci->get_topic_name());

//   // Set up camera manager
//   RCLCPP_INFO(get_logger(), "Starting camera manager and looking for cameras...");
//   camera_manager.start();

//   if (camera_manager.cameras().empty()) {
//     RCLCPP_ERROR(get_logger(), "No cameras detected by libcamera");
//     throw std::runtime_error("No cameras available");
//   }

//   RCLCPP_INFO(get_logger(), "Found %zu camera(s):", camera_manager.cameras().size());
//   for (const auto &cam : camera_manager.cameras()) {
//     RCLCPP_INFO(get_logger(), "  - Camera ID: %s", cam->id().c_str());
//   }

//   // Get camera by ID or use the first one
//   if (!camera_id.empty()) {
//     RCLCPP_INFO(get_logger(), "Trying to use specified camera: %s", camera_id.c_str());
//     camera = camera_manager.get(camera_id);
//     if (!camera) {
//       RCLCPP_ERROR(get_logger(), "Camera with ID '%s' not found", camera_id.c_str());
//       throw std::runtime_error("Camera not found");
//     }
//   }
//   else {
//     camera = camera_manager.cameras().front();
//     RCLCPP_INFO(get_logger(), "Using default camera: %s", camera->id().c_str());
//   }

//   if (camera->acquire()) {
//     throw std::runtime_error("Failed to acquire camera");
//   }

//   // Configure camera for RGB888 stream
//   RCLCPP_INFO(get_logger(), "Configuring camera with width=%d, height=%d, format=RGB888",
//               size.width, size.height);

//   std::unique_ptr<libcamera::CameraConfiguration> cfg =
//     camera->generateConfiguration({libcamera::StreamRole::Viewfinder});

//   if (!cfg) {
//     RCLCPP_ERROR(get_logger(), "Failed to generate camera configuration");
//     throw std::runtime_error("Failed to generate configuration");
//   }

//   libcamera::StreamConfiguration &scfg = cfg->at(0);
//   RCLCPP_INFO(get_logger(), "Default configuration: %s", scfg.toString().c_str());

//   // Set our desired format
//   scfg.pixelFormat = libcamera::formats::RGB888;
//   scfg.size = size;

//   // Validate configuration
//   auto validation_status = cfg->validate();
//   if (validation_status == libcamera::CameraConfiguration::Invalid) {
//     RCLCPP_ERROR(get_logger(), "Invalid camera configuration");
//     throw std::runtime_error("Invalid camera configuration");
//   }
//   else if (validation_status == libcamera::CameraConfiguration::Adjusted) {
//     RCLCPP_WARN(get_logger(), "Stream configuration adjusted to: %s", scfg.toString().c_str());
//   }

//   if (camera->configure(cfg.get()) < 0) {
//     RCLCPP_ERROR(get_logger(), "Failed to apply camera configuration");
//     throw std::runtime_error("Failed to configure camera");
//   }

//   RCLCPP_INFO(get_logger(), "Camera successfully configured with %s stream", scfg.toString().c_str());

//   // Set up camera info manager
//   std::string cname = camera->id() + '_' + scfg.size.toString();
//   const auto &props = camera->properties();
//   if (auto model = props.get(libcamera::properties::Model)) {
//     cname = model.value() + '_' + cname;
//   }

//   // Clean camera name
//   for (auto &c : cname) {
//     if (!std::isalnum(c))
//       c = '_';
//   }

//   cim.setCameraName(cname);
//   const std::string &camera_info_url = declare_parameter<std::string>("camera_info_url", "");
//   if (!cim.loadCameraInfo(camera_info_url)) {
//     RCLCPP_WARN(get_logger(), "Failed to load camera calibration from URL");
//   }

//   // Allocate buffers
//   stream = scfg.stream();
//   allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
//   allocator->allocate(stream);

//   // Create requests
//   for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
//     std::unique_ptr<libcamera::Request> request = camera->createRequest();
//     if (!request) {
//       throw std::runtime_error("Failed to create request");
//     }

//     // Map buffer memory
//     size_t buffer_length = 0;
//     int fd = -1;
//     for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
//       buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
//       if (fd == -1) {
//         fd = plane.fd.get();
//       }
//     }

//     void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);
//     if (data == MAP_FAILED) {
//       throw std::runtime_error("mmap failed");
//     }
//     buffer_info[buffer.get()] = {data, buffer_length};

//     if (request->addBuffer(stream, buffer.get()) < 0) {
//       throw std::runtime_error("Failed to set buffer for request");
//     }

//     requests.push_back(std::move(request));
//   }

//   // Register callback
//   camera->requestCompleted.connect(this, &MinimalCameraNode::requestComplete);

//   // Start camera with initial controls
//   libcamera::ControlList controls;
//   controls.set(libcamera::controls::AeEnable, AE_ENABLE);
//   controls.set(libcamera::controls::ExposureTime, EXPOSURE_TIME_US);

//   RCLCPP_INFO(get_logger(), "Setting camera exposure: AE_ENABLE=%s, EXPOSURE_TIME_US=%u",
//               AE_ENABLE ? "true" : "false", EXPOSURE_TIME_US);

//   RCLCPP_INFO(get_logger(), "Starting camera capture...");
//   if (camera->start(&controls)) {
//     RCLCPP_ERROR(get_logger(), "Failed to start camera");
//     throw std::runtime_error("Failed to start camera");
//   }

//   RCLCPP_INFO(get_logger(), "Camera started successfully");
//   running = true;

//   // Queue all requests
//   for (std::unique_ptr<libcamera::Request> &request : requests) {
//     camera->queueRequest(request.get());
//   }

//   stats_timer = create_wall_timer(
//     std::chrono::seconds(10),
//     std::bind(&MinimalCameraNode::printTimestampStats, this));
// }

// // Destructor
// MinimalCameraNode::~MinimalCameraNode()
// {
//   running = false;

//   if (camera) {
//     camera->stop();
//     allocator->free(stream);
//     allocator.reset();
//     camera->release();
//     camera.reset();
//   }

//   camera_manager.stop();

//   // Clean up memory mapped buffers
//   for (const auto &e : buffer_info) {
//     if (munmap(e.second.data, e.second.size) == -1) {
//       RCLCPP_ERROR(get_logger(), "munmap failed");
//     }
//   }

//   // Clean up timeshare mapping
//   if (pointt != (time_stamp *)MAP_FAILED) {
//     if (munmap(pointt, sizeof(time_stamp)) == -1) {
//       RCLCPP_ERROR(get_logger(), "munmap failed for timeshare");
//     }
//   }
// }

// // Flip RGB888 image vertically and horizontally
// void
// MinimalCameraNode::flipRGB888Image(uint8_t *data, int width, int height, int step)
// {
//   // Create a temporary buffer for a single row
//   std::vector<uint8_t> temp_row(step);

//   // Flip vertically (swap rows from top and bottom)
//   for (int y = 0; y < height / 2; y++) {
//     uint8_t *top_row = data + y * step;
//     uint8_t *bottom_row = data + (height - 1 - y) * step;

//     // Swap entire rows
//     memcpy(temp_row.data(), top_row, step);
//     memcpy(top_row, bottom_row, step);
//     memcpy(bottom_row, temp_row.data(), step);
//   }

//   // Flip horizontally (RGB pixels within each row)
//   for (int y = 0; y < height; y++) {
//     uint8_t *row = data + y * step;
//     for (int x = 0; x < width / 2; x++) {
//       // Swap RGB values (3 bytes per pixel)
//       std::swap(row[x * 3], row[(width - 1 - x) * 3]);         // R
//       std::swap(row[x * 3 + 1], row[(width - 1 - x) * 3 + 1]); // G
//       std::swap(row[x * 3 + 2], row[(width - 1 - x) * 3 + 2]); // B
//     }
//   }
// }

// void
// MinimalCameraNode::requestComplete(libcamera::Request *request)
// {
//   if (request->status() == libcamera::Request::RequestComplete) {
//     // Check if there are any subscribers before processing
//     bool has_raw_subscribers = pub_image->get_subscription_count() > 0;
//     bool has_camera_info_subscribers = pub_ci->get_subscription_count() > 0;

//     // Get buffer and metadata
//     const libcamera::FrameBuffer *buffer = request->findBuffer(stream);
//     const libcamera::FrameMetadata &metadata = buffer->metadata();

//     // Get camera timestamp from metadata (in ns)
//     int64_t camera_timestamp_ns = 0;
//     for (const auto &plane : metadata.planes()) {
//       if (plane.bytesused > 0) {
//         camera_timestamp_ns = metadata.timestamp;
//         break;
//       }
//     }

//     // Debug output for frames
//     static int frame_count = 0;
//     frame_count++;

//     // Current system time for comparison
//     rclcpp::Time system_time = this->now();
//     int64_t system_time_ns = system_time.nanoseconds();

//     // Hardware timestamp from shared memory
//     int64_t hw_timestamp_ns = 0;
//     bool hw_timestamp_valid = false;

//     if (pointt != (time_stamp *)MAP_FAILED) {
//       hw_timestamp_ns = pointt->low;
//       hw_timestamp_valid = (hw_timestamp_ns != 0);

//       // For debug: read directly from file periodically
//       if (frame_count % 100 == 1) {
//         int fd = open(timeshare_path.c_str(), O_RDONLY);
//         if (fd >= 0) {
//           time_stamp buffer;
//           if (read(fd, &buffer, sizeof(time_stamp)) == sizeof(time_stamp)) {
//             RCLCPP_INFO(get_logger(), "Direct file read - high: %ld, low: %ld",
//                         buffer.high, buffer.low);
//           }
//           close(fd);
//         }
//       }
//     }

//     // Prepare header and timestamp
//     std_msgs::msg::Header hdr;
//     hdr.frame_id = "camera";
//     const libcamera::StreamConfiguration &cfg = stream->configuration();

//     // Format the LiDAR HW timestamp properly for ROS messages
//     if (hw_timestamp_valid) {
//       // Convert to ROS time:
//       // The LiDAR puts nanoseconds since Unix epoch in pointt->low
//       // This is exactly what ROS2 expects for its time representation
//       hdr.stamp.sec = hw_timestamp_ns / 1000000000;
//       hdr.stamp.nanosec = hw_timestamp_ns % 1000000000;

//       // Track timestamp differences for debugging
//       static int64_t last_hw_timestamp = 0;
//       if (last_hw_timestamp > 0) {
//         double time_diff_ms = (hw_timestamp_ns - last_hw_timestamp) / 1000000.0;
//         if (std::abs(time_diff_ms - 100.0) > 10.0) {
//           RCLCPP_WARN(get_logger(), "[Frame %d] Unusual timestamp jump: %.2f ms between HW timestamps",
//                       frame_count, time_diff_ms);
//         }
//       }
//       last_hw_timestamp = hw_timestamp_ns;
//     }
//     else if (camera_timestamp_ns > 0) {
//       // Fall back to camera timestamp if available
//       hdr.stamp.sec = camera_timestamp_ns / 1000000000;
//       hdr.stamp.nanosec = camera_timestamp_ns % 1000000000;
//     }
//     else {
//       // Last resort is system time
//       hdr.stamp = system_time;
//     }

//     double hw_sec = hw_timestamp_valid ? (hw_timestamp_ns / 1.0e9) : 0.0;
//     double camera_sec = camera_timestamp_ns / 1.0e9;
//     double system_sec = system_time_ns / 1.0e9;

//     // Publish data if there are subscribers
//     if (has_raw_subscribers || has_camera_info_subscribers) {
//       // Publish raw image if there are subscribers
//       if (has_raw_subscribers) {
//         auto msg_img = std::make_unique<sensor_msgs::msg::Image>();
//         msg_img->header = hdr;
//         msg_img->width = cfg.size.width;
//         msg_img->height = cfg.size.height;
//         msg_img->step = cfg.stride;
//         msg_img->encoding = "rgb8";
//         msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);

//         msg_img->data.resize(buffer_info[buffer].size);
//         memcpy(msg_img->data.data(), buffer_info[buffer].data, buffer_info[buffer].size);

//         if (flip_image) {
//           flipRGB888Image(msg_img->data.data(), cfg.size.width, cfg.size.height, cfg.stride);
//         }

//         pub_image->publish(std::move(msg_img));
//       }

//       // Publish camera info if there are subscribers
//       if (has_camera_info_subscribers) {
//         sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
//         ci.header = hdr;
//         pub_ci->publish(ci);
//       }
//     }
//   }
//   else {
//     RCLCPP_WARN(get_logger(), "Request returned with status: %d", (int)request->status());
//   }

//   // Queue the request again with the same control values
//   request->reuse(libcamera::Request::ReuseBuffers);
//   request->controls().set(libcamera::controls::AeEnable, AE_ENABLE);
//   request->controls().set(libcamera::controls::ExposureTime, EXPOSURE_TIME_US);
//   camera->queueRequest(request);
// }

// bool
// MinimalCameraNode::setupTimeshare()
// {
//   RCLCPP_INFO(get_logger(), "Setting up timeshare at path: %s", timeshare_path.c_str());

//   // Clean up previous mapping if it exists
//   if (pointt != (time_stamp *)MAP_FAILED) {
//     munmap(pointt, sizeof(time_stamp));
//     pointt = (time_stamp *)MAP_FAILED;
//   }

//   // Try to open the timeshare file
//   int fd = open(timeshare_path.c_str(), O_RDWR);
//   if (fd < 0) {
//     RCLCPP_ERROR(get_logger(), "Could not open timeshare file %s: %s",
//                  timeshare_path.c_str(), strerror(errno));
//     RCLCPP_INFO(get_logger(), "Will use system time for timestamps");
//     return false;
//   }

//   // Check if file exists and is appropriate size
//   struct stat file_stat;
//   if (fstat(fd, &file_stat) < 0) {
//     RCLCPP_WARN(get_logger(), "Could not get timeshare file stats: %s", strerror(errno));
//   }
//   else {
//     RCLCPP_INFO(get_logger(), "Timeshare file size: %ld bytes", file_stat.st_size);

//     // The file is likely a sparse file with holes. For sparse files, st_size might report
//     // a smaller size than what's needed for mapping the full time_stamp structure.
//     // We'll accept as long as there's enough space for the "low" timestamp (8 bytes)
//     if (file_stat.st_size < 16) { // Minimum size needed for high and low values
//       RCLCPP_ERROR(get_logger(),
//                    "Timeshare file size too small: %ld bytes (expected at least 16 bytes)",
//                    file_stat.st_size);
//       close(fd);
//       return false;
//     }
//   }

//   // Map the shared memory
//   pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
//                               MAP_SHARED, fd, 0);
//   close(fd); // File descriptor can be closed after mapping

//   if (pointt == (time_stamp *)MAP_FAILED) {
//     RCLCPP_ERROR(get_logger(), "mmap failed for timeshare file: %s", strerror(errno));
//     return false;
//   }

//   // Output initial timestamp values
//   RCLCPP_INFO(get_logger(), "Successfully mapped timeshare file for hardware synchronization");
//   RCLCPP_INFO(get_logger(), "Initial timeshare values - high: %ld, low: %ld",
//               pointt->high, pointt->low);

//   // Convert the LiDAR timestamp to a human-readable date for verification
//   if (pointt->low != 0) {
//     time_t seconds = pointt->low / 1000000000;
//     char time_buf[32];
//     struct tm tm_info;

//     // Convert to local time
//     localtime_r(&seconds, &tm_info);
//     strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);

//     RCLCPP_INFO(get_logger(), "LiDAR timestamp in human-readable form: %s.%09ld",
//                 time_buf, pointt->low % 1000000000);

//     // Compare to system time
//     auto current_time = this->now();
//     seconds = current_time.seconds();
//     localtime_r(&seconds, &tm_info);
//     strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);

//     RCLCPP_INFO(get_logger(), "System time in human-readable form: %s.%09ld",
//                 time_buf, current_time.nanoseconds() % 1000000000);

//     double diff_seconds = std::abs(double(pointt->low) / 1.0e9 - double(current_time.nanoseconds()) / 1.0e9);
//     RCLCPP_INFO(get_logger(), "Time difference between LiDAR and system: %.3f seconds", diff_seconds);

//     if (diff_seconds > 3600) {
//       RCLCPP_WARN(get_logger(), "LiDAR and system times differ by more than an hour!");
//       RCLCPP_WARN(get_logger(), "This may indicate different time epochs are being used.");
//     }
//   }
//   else {
//     RCLCPP_WARN(get_logger(), "Initial LiDAR timestamp is zero. Waiting for LiDAR to update timestamp...");
//   }

//   return true;
// }
// } // namespace camera

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(camera::MinimalCameraNode)
