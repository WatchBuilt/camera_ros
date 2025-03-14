// #include "ParameterHandler.hpp"
// #include "format_mapping.hpp"
// #include "pretty_print.hpp"
// #include <algorithm>
// #include <camera_info_manager/camera_info_manager.hpp>
// #include <cassert>
// #include <cctype>
// #include <cerrno>
// #include <cstdint>
// #include <cstring>
// #if __has_include(<cv_bridge/cv_bridge.hpp>)
// #include <cv_bridge/cv_bridge.hpp>
// #elif __has_include(<cv_bridge/cv_bridge.h>)
// #include <cv_bridge/cv_bridge.h>
// #endif
// #include <atomic>
// #include <chrono>
// #include <fcntl.h>
// #include <iostream>
// #include <libcamera/base/shared_fd.h>
// #include <libcamera/base/signal.h>
// #include <libcamera/base/span.h>
// #include <libcamera/camera.h>
// #include <libcamera/camera_manager.h>
// #include <libcamera/control_ids.h>
// #include <libcamera/formats.h>
// #include <libcamera/framebuffer.h>
// #include <libcamera/framebuffer_allocator.h>
// #include <libcamera/geometry.h>
// #include <libcamera/pixel_format.h>
// #include <libcamera/property_ids.h>
// #include <libcamera/request.h>
// #include <libcamera/stream.h>
// #include <memory>
// #include <optional>
// #include <queue>
// #include <rcl/context.h>
// #include <rcl_interfaces/msg/floating_point_range.hpp>
// #include <rcl_interfaces/msg/integer_range.hpp>
// #include <rcl_interfaces/msg/parameter_descriptor.hpp>
// #include <rcl_interfaces/msg/parameter_type.hpp>
// #include <rcl_interfaces/msg/set_parameters_result.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rclcpp/node_interfaces/node_parameters_interface.hpp>
// #include <rclcpp/parameter.hpp>
// #include <rclcpp/parameter_value.hpp>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/time.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <shared_mutex>
// #include <std_msgs/msg/header.hpp>
// #include <stdexcept>
// #include <string>
// #include <sys/mman.h>
// #include <thread>
// #include <unistd.h>
// #include <unordered_map>
// #include <utility>
// #include <vector>

// // SIMD support
// #ifdef __SSE2__
// #include <emmintrin.h>
// #endif

// namespace rclcpp
// {
// class NodeOptions;
// }

// namespace camera
// {

// // Throttled logging macros
// #define CAMERA_LOG_DEBUG_THROTTLE(logger, interval_ms, ...)                                                  \
//   do {                                                                                                       \
//     static auto last_log_time = std::chrono::steady_clock::time_point();                                     \
//     auto now = std::chrono::steady_clock::now();                                                             \
//     if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= interval_ms) { \
//       RCLCPP_DEBUG(logger, __VA_ARGS__);                                                                     \
//       last_log_time = now;                                                                                   \
//     }                                                                                                        \
//   } while (0)

// #define CAMERA_LOG_INFO_THROTTLE(logger, interval_ms, ...)                                                   \
//   do {                                                                                                       \
//     static auto last_log_time = std::chrono::steady_clock::time_point();                                     \
//     auto now = std::chrono::steady_clock::now();                                                             \
//     if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= interval_ms) { \
//       RCLCPP_INFO(logger, __VA_ARGS__);                                                                      \
//       last_log_time = now;                                                                                   \
//     }                                                                                                        \
//   } while (0)

// // Shared timestamp structure for hardware sync
// struct alignas(64) time_stamp
// {
//   int64_t high;
//   int64_t low;
// };

// class CameraNode : public rclcpp::Node
// {
// public:
//   explicit CameraNode(const rclcpp::NodeOptions &options);
//   ~CameraNode();

//   // Public methods for buffer pool management
//   void
//   releaseBufferToPool(std::vector<uint8_t> *buffer_ptr);

// private:
//   libcamera::CameraManager camera_manager;
//   std::shared_ptr<libcamera::Camera> camera;
//   libcamera::Stream *stream;
//   std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
//   std::vector<std::unique_ptr<libcamera::Request>> requests;

//   // Thread pool instead of per-request threads
//   const size_t THREAD_POOL_SIZE = 4; // Adjust based on CPU cores
//   std::vector<std::thread> thread_pool;
//   std::queue<libcamera::Request *> request_queue;
//   std::mutex queue_mutex;
//   std::condition_variable queue_condvar;

//   alignas(64) std::atomic<bool> running;
//   bool flip_image = true;

//   struct buffer_info_t
//   {
//     void *data;
//     size_t size;
//   };
//   std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info;
//   std::shared_mutex buffer_mutex; // Reader-writer lock for buffer access

//   // Buffer pool for image processing
//   struct ImageBuffer
//   {
//     std::vector<uint8_t> data;
//     bool in_use;
//   };
//   std::vector<ImageBuffer> image_buffer_pool;
//   std::mutex buffer_pool_mutex;
//   const size_t MAX_BUFFER_POOL_SIZE = 3;

//   // Pre-allocated RGB buffer for flipping
//   alignas(64) std::vector<uint8_t> rgb_flip_buffer;
//   std::mutex rgb_buffer_mutex;

//   // Reusable message objects
//   sensor_msgs::msg::Image rgb_image_msg;
//   sensor_msgs::msg::CameraInfo reusable_ci_msg;

//   // timestamp offset (ns) from camera time to system time
//   int64_t time_offset = 0;

//   // Hardware sync related members
//   alignas(64) time_stamp *pointt = (time_stamp *)MAP_FAILED;
//   bool trigger_enable = true;
//   std::string timeshare_path;
//   bool using_hw_timestamp = false;
//   std::atomic<bool> check_timeshare_periodically = true;
//   std::thread timeshare_check_thread;

//   // Optimized timestamp function using function pointer
//   std::function<void(std_msgs::msg::Header &, const libcamera::FrameMetadata &)> timestampFunction;

//   const bool AE_ENABLE = false;
//   const uint32_t EXPOSURE_TIME_US = 90000; // exposure in µs

//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
//   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;

//   camera_info_manager::CameraInfoManager cim;
//   ParameterHandler parameter_handler;

//   void
//   requestComplete(libcamera::Request *const request);
//   void
//   threadPoolWorker();
//   void
//   processRequest(libcamera::Request *const request);
//   void
//   postParameterChange(const std::vector<rclcpp::Parameter> &parameters);
//   bool
//   setupTimeshare();
//   void
//   setupTimestampFunction();
//   void
//   checkTimeshareAvailability();
//   void
//   monitorTimeshareContent();

//   // Optimized RGB888 methods
//   void
//   initBufferPool();
//   void
//   initRGB888Memory();
//   std::vector<uint8_t> *
//   getBufferFromPool(size_t required_size);
//   void
//   publishRGB888ZeroCopy(libcamera::Request *request,
//                         const libcamera::FrameBuffer *buffer,
//                         const libcamera::StreamConfiguration &cfg,
//                         const std_msgs::msg::Header &hdr);
//   void
//   flipRGB888Image(uint8_t *data, int width, int height, int step);

//   // SIMD optimized methods
// #ifdef __SSE2__
//   void
//   flipRGB888ImageSIMD(uint8_t *data, int width, int height, int step);
//   void
//   optimizedCopy(void *dst, const void *src, size_t size);
// #else
//   void
//   optimizedCopy(void *dst, const void *src, size_t size)
//   {
//     memcpy(dst, src, size);
//   }
// #endif

//   // Helper methods
//   bool
//   isRGB888Format(const std::string &encoding, const libcamera::PixelFormat &pixelFormat)
//   {
//     return encoding == "rgb8" ||
//            pixelFormat == libcamera::formats::RGB888 ||
//            pixelFormat.toString() == "RGB888";
//   }

//   void
//   prefetchRGBData(const void *addr, size_t size)
//   {
//     const char *ptr = static_cast<const char *>(addr);
//     // Prefetch in cache line sized chunks (typically 64 bytes)
//     for (size_t i = 0; i < size; i += 64) {
//       __builtin_prefetch(ptr + i, 0, 3); // Read, high temporal locality
//     }
//   }
// };

// RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)

// // Optimized RGB888 image flipping
// void
// CameraNode::flipRGB888Image(uint8_t *data, int width, int height, int step)
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

// #ifdef __SSE2__
// // SIMD-optimized version for RGB888
// void
// CameraNode::flipRGB888ImageSIMD(uint8_t *data, int width, int height, int step)
// {
//   // Vertical flip remains the same (row swapping)
//   std::vector<uint8_t> temp_row(step);
//   for (int y = 0; y < height / 2; y++) {
//     uint8_t *top_row = data + y * step;
//     uint8_t *bottom_row = data + (height - 1 - y) * step;

//     std::memcpy(temp_row.data(), top_row, step);
//     std::memcpy(top_row, bottom_row, step);
//     std::memcpy(bottom_row, temp_row.data(), step);
//   }

//   // For horizontal flip, we need special handling for RGB
//   // Since we can't easily use SIMD for 3-byte (RGB) pixels, we'll use regular swap
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

// // Optimized memory copy with SIMD
// void
// CameraNode::optimizedCopy(void *dst, const void *src, size_t size)
// {
//   if (size >= 16 &&
//       reinterpret_cast<uintptr_t>(dst) % 16 == 0 &&
//       reinterpret_cast<uintptr_t>(src) % 16 == 0) {
//     // Use SSE2 for aligned copies
//     size_t blocks = size / 16;
//     __m128i *d = reinterpret_cast<__m128i *>(dst);
//     const __m128i *s = reinterpret_cast<const __m128i *>(src);

//     for (size_t i = 0; i < blocks; ++i) {
//       _mm_store_si128(&d[i], _mm_load_si128(&s[i]));
//     }

//     // Handle remaining bytes
//     size_t remaining = size % 16;
//     if (remaining > 0) {
//       memcpy(reinterpret_cast<char *>(dst) + size - remaining,
//              reinterpret_cast<const char *>(src) + size - remaining,
//              remaining);
//     }
//   }
//   else {
//     // Fall back to regular memcpy for unaligned data
//     memcpy(dst, src, size);
//   }
// }
// #endif

// libcamera::StreamRole
// get_role(const std::string &role)
// {
//   static const std::unordered_map<std::string, libcamera::StreamRole> roles_map = {
//     {"raw", libcamera::StreamRole::Raw},
//     {"still", libcamera::StreamRole::StillCapture},
//     {"video", libcamera::StreamRole::VideoRecording},
//     {"viewfinder", libcamera::StreamRole::Viewfinder},
//   };

//   try {
//     return roles_map.at(role);
//   }
//   catch (const std::out_of_range &) {
//     throw std::runtime_error("invalid stream role: \"" + role + "\"");
//   }
// }

// // Initialize buffer pool
// void
// CameraNode::initBufferPool()
// {
//   image_buffer_pool.resize(MAX_BUFFER_POOL_SIZE);
//   for (auto &buffer : image_buffer_pool) {
//     buffer.in_use = false;
//   }
// }

// // Initialize RGB888 specific memory
// void
// CameraNode::initRGB888Memory()
// {
//   // Pre-allocate with alignment for common HD resolution
//   rgb_flip_buffer.reserve(1920 * 1080 * 3);

//   // Pre-setup image message fields that don't change
//   rgb_image_msg.encoding = "rgb8";
//   rgb_image_msg.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
// }

// // Get buffer from pool
// std::vector<uint8_t> *
// CameraNode::getBufferFromPool(size_t required_size)
// {
//   std::lock_guard<std::mutex> lock(buffer_pool_mutex);

//   // Find an available buffer
//   for (auto &buffer : image_buffer_pool) {
//     if (!buffer.in_use) {
//       // Resize if needed
//       if (buffer.data.size() < required_size) {
//         buffer.data.resize(required_size);
//       }
//       buffer.in_use = true;
//       return &buffer.data;
//     }
//   }

//   // If no buffer is available, create a new one if pool is not full
//   if (image_buffer_pool.size() < MAX_BUFFER_POOL_SIZE) {
//     image_buffer_pool.push_back({std::vector<uint8_t>(required_size), true});
//     return &image_buffer_pool.back().data;
//   }

//   // If pool is full and no buffer is available, return nullptr
//   return nullptr;
// }

// // Release buffer back to pool
// void
// CameraNode::releaseBufferToPool(std::vector<uint8_t> *buffer_ptr)
// {
//   std::lock_guard<std::mutex> lock(buffer_pool_mutex);

//   // Find the buffer in the pool and mark it as available
//   for (auto &buffer : image_buffer_pool) {
//     if (&buffer.data == buffer_ptr) {
//       buffer.in_use = false;
//       return;
//     }
//   }
// }

// // Setup the shared memory for hardware timing
// bool
// CameraNode::setupTimeshare()
// {
//   // Clean up previous mapping if it exists
//   if (pointt != (time_stamp *)MAP_FAILED) {
//     munmap(pointt, sizeof(time_stamp));
//     pointt = (time_stamp *)MAP_FAILED;
//     using_hw_timestamp = false;
//   }

//   // Try to open the timeshare file
//   int fd = open(timeshare_path.c_str(), O_RDWR);
//   if (fd < 0) {
//     CAMERA_LOG_DEBUG_THROTTLE(get_logger(), 5000, "Could not open timeshare file %s: %s",
//                               timeshare_path.c_str(), strerror(errno));
//     return false;
//   }

//   // Map the shared memory
//   pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
//                               MAP_SHARED, fd, 0);
//   close(fd); // File descriptor can be closed after mapping

//   if (pointt == (time_stamp *)MAP_FAILED) {
//     RCLCPP_WARN(get_logger(), "mmap failed for timeshare file: %s", strerror(errno));
//     return false;
//   }

//   // Log initial values in shared memory
//   RCLCPP_INFO(get_logger(), "Successfully mapped timeshare file for hardware synchronization");
//   RCLCPP_INFO(get_logger(), "Initial timeshare values - high: %ld, low: %ld",
//               pointt->high, pointt->low);

//   using_hw_timestamp = true;

//   // Set up the timestamp function
//   setupTimestampFunction();

//   // Start a monitoring thread to periodically print timeshare contents
//   std::thread monitoring_thread(&CameraNode::monitorTimeshareContent, this);
//   monitoring_thread.detach(); // Let it run independently

//   return true;
// }

// // Setup optimized timestamp function
// void
// CameraNode::setupTimestampFunction()
// {
//   if (trigger_enable && pointt != (time_stamp *)MAP_FAILED) {
//     // Use hardware timestamp function
//     timestampFunction = [this](std_msgs::msg::Header &hdr, const libcamera::FrameMetadata &metadata) {
//       int64_t timestamp_ns = pointt->low;
//       if (timestamp_ns != 0) {
//         hdr.stamp.sec = timestamp_ns / 1000000000;
//         hdr.stamp.nanosec = timestamp_ns % 1000000000;
//       }
//       else {
//         // Fallback to camera timestamp with offset
//         hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
//       }
//     };
//     RCLCPP_INFO(get_logger(), "Using hardware timestamp function");
//   }
//   else {
//     // Use camera timestamp function
//     timestampFunction = [this](std_msgs::msg::Header &hdr, const libcamera::FrameMetadata &metadata) {
//       hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
//     };
//     RCLCPP_INFO(get_logger(), "Using camera timestamp function");
//   }
// }

// // Monitor the content of the timeshare file to verify it's being updated
// void
// CameraNode::monitorTimeshareContent()
// {
//   int64_t last_low = 0;
//   int check_count = 0;

//   while (running && pointt != (time_stamp *)MAP_FAILED) {
//     // Only log when value changes or on periodic intervals (less frequently)
//     if (pointt->low != last_low) {
//       // Only log significant changes (greater than 5ms difference)
//       if (abs(pointt->low - last_low) > 5000000) {
//         CAMERA_LOG_DEBUG_THROTTLE(get_logger(), 5000, "Timeshare updated - previous: %ld, current: %ld",
//                                   last_low, pointt->low);
//       }
//       last_low = pointt->low;
//     }
//     else if (check_count % 20 == 0) { // Log less frequently (every 20 checks)
//       CAMERA_LOG_DEBUG_THROTTLE(get_logger(), 5000, "Timeshare value: %ld", pointt->low);
//     }

//     check_count++;
//     std::this_thread::sleep_for(std::chrono::seconds(1)); // Check less frequently
//   }
// }

// // Periodically check if timeshare file becomes available
// void
// CameraNode::checkTimeshareAvailability()
// {
//   while (check_timeshare_periodically && rclcpp::ok()) {
//     // If we're not using hardware timestamp yet, try to set it up
//     if (!using_hw_timestamp && trigger_enable) {
//       if (setupTimeshare()) {
//         RCLCPP_INFO(get_logger(), "Timeshare file now available. Using hardware timestamps.");
//       }
//     }
//     // Sleep for 5 seconds before checking again
//     std::this_thread::sleep_for(std::chrono::seconds(5));
//   }
// }
// void
// CameraNode::requestComplete(libcamera::Request *const request)
// {
//   std::unique_lock<std::mutex> lock(queue_mutex);
//   request_queue.push(request);
//   queue_condvar.notify_one();
// }

// void
// CameraNode::threadPoolWorker()
// {
//   while (running) {
//     libcamera::Request *request = nullptr;

//     // Wait for a request
//     {
//       std::unique_lock<std::mutex> lock(queue_mutex);
//       queue_condvar.wait(lock, [this] {
//         return !request_queue.empty() || !running;
//       });

//       if (!running)
//         return;

//       if (request_queue.empty())
//         continue;

//       request = request_queue.front();
//       request_queue.pop();
//     }

//     // Process the request - for now, use the original process method with minimal changes
//     if (request->status() == libcamera::Request::RequestComplete) {
//       assert(request->buffers().size() == 1);

//       // Check if there are any subscribers before processing
//       bool has_raw_subscribers = pub_image->get_subscription_count() > 0;
//       bool has_camera_info_subscribers = pub_ci->get_subscription_count() > 0;

//       // If no subscribers, skip all processing
//       if (!has_raw_subscribers && !has_camera_info_subscribers) {
//         // Skip to request reuse
//         request->reuse(libcamera::Request::ReuseBuffers);
//         parameter_handler.move_control_values(request->controls());
//         request->controls().set(libcamera::controls::AeEnable, AE_ENABLE);
//         request->controls().set(libcamera::controls::ExposureTime, EXPOSURE_TIME_US);
//         camera->queueRequest(request);
//         continue;
//       }

//       // get the stream and buffer from the request
//       const libcamera::FrameBuffer *buffer = request->findBuffer(stream);
//       const libcamera::FrameMetadata &metadata = buffer->metadata();
//       size_t bytesused = 0;
//       for (const libcamera::FrameMetadata::Plane &plane : metadata.planes())
//         bytesused += plane.bytesused;

//       // set time offset once for accurate timing using the device time
//       if (time_offset == 0)
//         time_offset = this->now().nanoseconds() - metadata.timestamp;

//       // prepare header and timestamp
//       std_msgs::msg::Header hdr;

//       // Use hardware timestamp if available
//       if (trigger_enable && pointt != (time_stamp *)MAP_FAILED && pointt->low != 0) {
//         // Use timestamp from shared memory (synchronized with hardware trigger)
//         int64_t timestamp_ns = pointt->low;

//         // Create ROS timestamp manually from the raw nanoseconds
//         hdr.stamp.sec = timestamp_ns / 1000000000;
//         hdr.stamp.nanosec = timestamp_ns % 1000000000;
//       }
//       else {
//         // Use camera timestamp with offset
//         hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
//       }

//       hdr.frame_id = "camera";
//       const libcamera::StreamConfiguration &cfg = stream->configuration();

//       // Publish raw image if there are subscribers
//       if (has_raw_subscribers) {
//         // Process RGB888 format images
//         if (isRGB888Format(get_ros_encoding(cfg.pixelFormat), cfg.pixelFormat)) {
//           std::unique_ptr<sensor_msgs::msg::Image> msg_img = std::make_unique<sensor_msgs::msg::Image>();

//           assert(buffer_info[buffer].size == bytesused);
//           msg_img->header = hdr;
//           msg_img->width = cfg.size.width;
//           msg_img->height = cfg.size.height;
//           msg_img->step = cfg.stride;
//           msg_img->encoding = "rgb8";
//           msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
//           msg_img->data.resize(buffer_info[buffer].size);

// // Optimized copy with SIMD if available
// #ifdef __SSE2__
//           optimizedCopy(msg_img->data.data(), buffer_info[buffer].data, buffer_info[buffer].size);
// #else
//           memcpy(msg_img->data.data(), buffer_info[buffer].data, buffer_info[buffer].size);
// #endif

//           if (flip_image) {
// // Use optimized RGB888 flipping
// #ifdef __SSE2__
//             flipRGB888ImageSIMD(msg_img->data.data(), cfg.size.width, cfg.size.height, cfg.stride);
// #else
//             flipRGB888Image(msg_img->data.data(), cfg.size.width, cfg.size.height, cfg.stride);
// #endif
//           }

//           pub_image->publish(std::move(msg_img));
//         }
//         else {
//           RCLCPP_ERROR_STREAM(get_logger(), "Unsupported pixel format: " +
//                                               stream->configuration().pixelFormat.toString() +
//                                               ". This node is optimized for RGB888 only.");
//         }
//       }

//       // Only publish camera info if there are subscribers
//       if (has_camera_info_subscribers) {
//         sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
//         ci.header = hdr;
//         pub_ci->publish(ci);
//       }
//     }
//     else if (request->status() == libcamera::Request::RequestCancelled) {
//       RCLCPP_ERROR_STREAM(get_logger(), "request '" << request->toString() << "' cancelled");
//     }

//     // redeclare implicitly undeclared parameters - only do this occasionally
//     static int param_counter = 0;
//     if ((++param_counter % 10) == 0) {
//       parameter_handler.redeclare();
//     }

//     // queue the request again for the next frame and update controls
//     request->reuse(libcamera::Request::ReuseBuffers);
//     parameter_handler.move_control_values(request->controls());

//     // Set exposure settings
//     request->controls().set(libcamera::controls::AeEnable, AE_ENABLE);
//     request->controls().set(libcamera::controls::ExposureTime, EXPOSURE_TIME_US);

//     camera->queueRequest(request);
//   }
// }

// // Constructor with RGB888 optimizations
// CameraNode::CameraNode(const rclcpp::NodeOptions &options)
//     : Node("camera", options),
//       cim(this),
//       parameter_handler(this)
// {
//   // Initialize optimized buffers and memory
//   initBufferPool();
//   initRGB888Memory();

//   // pixel format - force RGB888 for optimized path
//   rcl_interfaces::msg::ParameterDescriptor param_descr_format;
//   param_descr_format.description = "pixel format of streaming buffer";
//   param_descr_format.read_only = true;
//   std::string format = declare_parameter<std::string>("format", "RGB888", param_descr_format);

//   // If format not explicitly specified as RGB888, force it
//   if (format != "RGB888") {
//     RCLCPP_WARN(get_logger(), "Format '%s' specified, but this node is optimized for RGB888. Forcing RGB888 format.",
//                 format.c_str());
//     format = "RGB888";
//   }

//   // stream role
//   rcl_interfaces::msg::ParameterDescriptor param_descr_role;
//   param_descr_role.description = "stream role";
//   param_descr_role.additional_constraints = "one of {raw, still, video, viewfinder}";
//   param_descr_role.read_only = true;
//   const libcamera::StreamRole role =
//     get_role(declare_parameter<std::string>("role", "viewfinder", param_descr_role));

//   // image dimensions
//   rcl_interfaces::msg::ParameterDescriptor param_descr_ro;
//   param_descr_ro.read_only = true;
//   const uint32_t w = declare_parameter<int64_t>("width", {}, param_descr_ro);
//   const uint32_t h = declare_parameter<int64_t>("height", {}, param_descr_ro);
//   const libcamera::Size size {w, h};

//   // Hardware trigger enable
//   rcl_interfaces::msg::ParameterDescriptor param_descr_trigger;
//   param_descr_trigger.description = "Enable hardware triggering and synchronized timestamps";
//   trigger_enable = declare_parameter<bool>("trigger_enable", true, param_descr_trigger);

//   // camera info file url
//   rcl_interfaces::msg::ParameterDescriptor param_descr_camera_info_url;
//   param_descr_camera_info_url.description = "camera calibration info file url";
//   param_descr_camera_info_url.read_only = true;

//   // camera ID
//   declare_parameter("camera", rclcpp::ParameterValue {}, param_descr_ro.set__dynamic_typing(true));

//   // Setup hardware synchronization
//   if (trigger_enable) {
//     // Set up path to shared timestamp file
//     const char *user_name = getlogin();
//     timeshare_path = "/home/" + std::string(user_name) + "/timeshare";
//     RCLCPP_INFO(get_logger(), "Hardware triggering enabled, using timeshare at: %s", timeshare_path.c_str());

//     // Try to set up the timeshare file
//     if (setupTimeshare()) {
//       RCLCPP_INFO(get_logger(), "Hardware timestamp synchronization initialized successfully");
//     }
//     else {
//       RCLCPP_WARN(get_logger(), "Timeshare file not available, will check periodically");
//       // Start a thread to periodically check for the timeshare file
//       timeshare_check_thread = std::thread(&CameraNode::checkTimeshareAvailability, this);
//     }
//   }
//   else {
//     // Set up timestamp function for non-hardware timestamps
//     setupTimestampFunction();
//   }

//   // publisher for raw image and camera info
//   pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10); // Increased queue size
//   pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

//   // start camera manager and check for cameras
//   camera_manager.start();
//   if (camera_manager.cameras().empty())
//     throw std::runtime_error("no cameras available");

//   // get the camera
//   switch (get_parameter("camera").get_type()) {
//   case rclcpp::ParameterType::PARAMETER_NOT_SET:
//     // use first camera as default
//     camera = camera_manager.cameras().front();
//     RCLCPP_INFO_STREAM(get_logger(), camera_manager);
//     RCLCPP_WARN_STREAM(get_logger(),
//                        "no camera selected, using default: \"" << camera->id() << "\"");
//     RCLCPP_WARN_STREAM(get_logger(), "set parameter 'camera' to silence this warning");
//     break;
//   case rclcpp::ParameterType::PARAMETER_INTEGER:
//   {
//     const size_t id = get_parameter("camera").as_int();
//     if (id >= camera_manager.cameras().size()) {
//       RCLCPP_INFO_STREAM(get_logger(), camera_manager);
//       throw std::runtime_error("camera with id " + std::to_string(id) + " does not exist");
//     }
//     camera = camera_manager.cameras().at(id);
//     RCLCPP_DEBUG_STREAM(get_logger(), "found camera by id: " << id);
//   } break;
//   case rclcpp::ParameterType::PARAMETER_STRING:
//   {
//     const std::string name = get_parameter("camera").as_string();
//     camera = camera_manager.get(name);
//     if (!camera) {
//       RCLCPP_INFO_STREAM(get_logger(), camera_manager);
//       throw std::runtime_error("camera with name " + name + " does not exist");
//     }
//     RCLCPP_DEBUG_STREAM(get_logger(), "found camera by name: \"" << name << "\"");
//   } break;
//   default:
//     RCLCPP_ERROR_STREAM(get_logger(), "unsupported camera parameter type: "
//                                         << get_parameter("camera").get_type_name());
//     break;
//   }

//   if (!camera)
//     throw std::runtime_error("failed to find camera");

//   if (camera->acquire())
//     throw std::runtime_error("failed to acquire camera");

//   // configure camera stream
//   std::unique_ptr<libcamera::CameraConfiguration> cfg =
//     camera->generateConfiguration({role});

//   if (!cfg)
//     throw std::runtime_error("failed to generate configuration");

//   assert(cfg->size() == 1);
//   libcamera::StreamConfiguration &scfg = cfg->at(0);

//   // Force RGB888 format for optimized path
//   const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(format);
//   if (!format_requested.isValid()) {
//     throw std::runtime_error("invalid pixel format: \"" + format + "\"");
//   }
//   scfg.pixelFormat = format_requested;

//   // Set requested size if specified
//   if (!size.isNull()) {
//     scfg.size = size;
//   }

//   // Store selected stream configuration
//   const libcamera::StreamConfiguration selected_scfg = scfg;

//   switch (cfg->validate()) {
//   case libcamera::CameraConfiguration::Valid:
//     break;
//   case libcamera::CameraConfiguration::Adjusted:
//     RCLCPP_WARN_STREAM(get_logger(), "stream configuration adjusted from \""
//                                        << selected_scfg.toString() << "\" to \"" << scfg.toString()
//                                        << "\"");
//     break;
//   case libcamera::CameraConfiguration::Invalid:
//     throw std::runtime_error("failed to validate stream configurations");
//     break;
//   }

//   switch (camera->configure(cfg.get())) {
//   case -ENODEV:
//     throw std::runtime_error("configure: camera has been disconnected from the system");
//   case -EACCES:
//     throw std::runtime_error("configure: camera is not in a state where it can be configured");
//   case -EINVAL:
//     throw std::runtime_error("configure: configuration \"" + scfg.toString() + "\" is not valid");
//   default:
//     RCLCPP_INFO_STREAM(get_logger(), "camera \"" << camera->id() << "\" configured with "
//                                                  << scfg.toString() << " stream");
//     break;
//   }

//   // format camera name for calibration file
//   const libcamera::ControlList &props = camera->properties();
//   std::string cname = camera->id() + '_' + scfg.size.toString();
//   const std::optional<std::string> model = props.get(libcamera::properties::Model);
//   if (model)
//     cname = model.value() + '_' + cname;

//   // clean camera name of non-alphanumeric characters
//   cname.erase(
//     std::remove_if(cname.begin(), cname.end(), [](const char &x) { return std::isspace(x); }),
//     cname.cend());
//   std::replace_if(
//     cname.begin(), cname.end(), [](const char &x) { return !std::isalnum(x); }, '_');

//   if (!cim.setCameraName(cname))
//     throw std::runtime_error("camera name must only contain alphanumeric characters");

//   const std::string &camera_info_url = declare_parameter<std::string>(
//     "camera_info_url", {}, param_descr_camera_info_url);
//   if (!cim.loadCameraInfo(camera_info_url)) {
//     if (!camera_info_url.empty()) {
//       RCLCPP_WARN_STREAM(get_logger(), "failed to load camera calibration info from provided URL, using default URL");
//       cim.loadCameraInfo({});
//     }
//   }

//   parameter_handler.declare(camera->controls());

//   // allocate stream buffers and create one request per buffer
//   stream = scfg.stream();

//   allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
//   allocator->allocate(stream);

//   for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
//     std::unique_ptr<libcamera::Request> request = camera->createRequest();
//     if (!request)
//       throw std::runtime_error("Can't create request");

//     // multiple planes of the same buffer use the same file descriptor
//     size_t buffer_length = 0;
//     int fd = -1;
//     for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
//       if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset)
//         throw std::runtime_error("invalid offset");
//       buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
//       if (!plane.fd.isValid())
//         throw std::runtime_error("file descriptor is not valid");
//       if (fd == -1)
//         fd = plane.fd.get();
//       else if (fd != plane.fd.get())
//         throw std::runtime_error("plane file descriptors differ");
//     }

//     // memory-map the frame buffer planes
//     void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);
//     if (data == MAP_FAILED)
//       throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
//     buffer_info[buffer.get()] = {data, buffer_length};

//     if (request->addBuffer(stream, buffer.get()) < 0)
//       throw std::runtime_error("Can't set buffer for request");

//     requests.push_back(std::move(request));
//   }

//   // Initialize thread pool
//   running = true;
//   thread_pool.reserve(THREAD_POOL_SIZE);
//   for (size_t i = 0; i < THREAD_POOL_SIZE; ++i) {
//     thread_pool.emplace_back(&CameraNode::threadPoolWorker, this);
//   }

//   // register callback
//   camera->requestCompleted.connect(this, &CameraNode::requestComplete);

//   // start camera with initial controls
//   libcamera::ControlList controls = parameter_handler.get_control_values();
//   controls.set(libcamera::controls::AeEnable, AE_ENABLE);
//   controls.set(libcamera::controls::ExposureTime, EXPOSURE_TIME_US); // exposure in µs

//   if (camera->start(&controls))
//     throw std::runtime_error("failed to start camera");

//   // queue all requests
//   for (std::unique_ptr<libcamera::Request> &request : requests)
//     camera->queueRequest(request.get());
// }

// CameraNode::~CameraNode()
// {
//   // stop request callbacks - fixed to avoid unused variable warning
//   for (auto &req : requests) {
//     camera->requestCompleted.disconnect(req.get());
//   }

//   // stop request processing threads
//   running = false;

//   // stop timeshare checking thread
//   check_timeshare_periodically = false;
//   if (timeshare_check_thread.joinable()) {
//     timeshare_check_thread.join();
//   }

//   // Signal all worker threads to exit
//   {
//     std::unique_lock<std::mutex> lock(queue_mutex);
//     // Clear any pending requests
//     while (!request_queue.empty()) {
//       request_queue.pop();
//     }
//   }
//   queue_condvar.notify_all();

//   // wait for all thread pool workers to finish
//   for (std::thread &thread : thread_pool) {
//     if (thread.joinable()) {
//       thread.join();
//     }
//   }

//   // stop camera
//   if (camera->stop())
//     std::cerr << "failed to stop camera" << std::endl;
//   allocator->free(stream);
//   allocator.reset();
//   camera->release();
//   camera.reset();
//   camera_manager.stop();

//   // Clean up memory mapped buffer
//   for (const auto &e : buffer_info)
//     if (munmap(e.second.data, e.second.size) == -1)
//       std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;

//   // Clean up timeshare mapping
//   if (pointt != (time_stamp *)MAP_FAILED) {
//     if (munmap(pointt, sizeof(time_stamp)) == -1)
//       std::cerr << "munmap failed for timeshare: " << std::strerror(errno) << std::endl;
//   }

//   // Clear buffer pools
//   {
//     std::lock_guard<std::mutex> lock(buffer_pool_mutex);
//     image_buffer_pool.clear();
//   }

//   {
//     std::lock_guard<std::mutex> lock(rgb_buffer_mutex);
//     rgb_flip_buffer.clear();
//   }
// }
// } // namespace camera
