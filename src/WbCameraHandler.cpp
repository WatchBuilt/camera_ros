#include "WbCameraHandler.h"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace camera
{

// Helper functions for logging
void
WbCameraHandler::logDebug(const std::string &message)
{
  if (log_callback) {
    log_callback(LogLevel::DEBUG, message);
  }
  else {
    std::cout << "[DEBUG] " << message << std::endl;
  }
}

void
WbCameraHandler::logInfo(const std::string &message)
{
  if (log_callback) {
    log_callback(LogLevel::INFO, message);
  }
  else {
    std::cout << "[INFO] " << message << std::endl;
  }
}

void
WbCameraHandler::logWarn(const std::string &message)
{
  if (log_callback) {
    log_callback(LogLevel::WARN, message);
  }
  else {
    std::cerr << "[WARN] " << message << std::endl;
  }
}

void
WbCameraHandler::logError(const std::string &message)
{
  if (log_callback) {
    log_callback(LogLevel::ERROR, message);
  }
  else {
    std::cerr << "[ERROR] " << message << std::endl;
  }
}

WbCameraHandler::WbCameraHandler()
    : stream(nullptr),
      camera_name(""),
      auto_exposure_enabled(false),
      exposure_time_us(90000),
      pointt((time_stamp *)MAP_FAILED),
      running(false)
{
  logInfo("WbCameraHandler created");
}

WbCameraHandler::~WbCameraHandler()
{
  logInfo("WbCameraHandler destructor called");
  stop();

  // Clean up timeshare mapping
  if (pointt != (time_stamp *)MAP_FAILED) {
    if (munmap(pointt, sizeof(time_stamp)) == -1) {
      logError("munmap failed for timeshare: " + std::string(strerror(errno)));
    }
    pointt = (time_stamp *)MAP_FAILED;
  }
  logInfo("WbCameraHandler destroyed");
}

bool
WbCameraHandler::initialize(const std::string &camera_id, uint32_t width, uint32_t height)
{
  logInfo("Initializing WbCameraHandler with camera_id='" + camera_id +
          "', width=" + std::to_string(width) + ", height=" + std::to_string(height));

  // Set up camera manager
  logInfo("Starting camera manager and looking for cameras...");
  try {
    camera_manager.start();
  }
  catch (const std::exception &e) {
    logError("Exception while starting camera manager: " + std::string(e.what()));
    return false;
  }
  catch (...) {
    logError("Unknown exception while starting camera manager");
    return false;
  }

  if (camera_manager.cameras().empty()) {
    logError("No cameras detected by libcamera");
    return false;
  }

  logInfo("Found " + std::to_string(camera_manager.cameras().size()) + " camera(s):");
  for (const auto &cam : camera_manager.cameras()) {
    logInfo("  - Camera ID: " + cam->id());
  }

  // Get camera by ID or use the first one
  if (!camera_id.empty()) {
    logInfo("Trying to use specified camera: " + camera_id);
    camera = camera_manager.get(camera_id);
    if (!camera) {
      logError("Camera with ID '" + camera_id + "' not found");
      return false;
    }
  }
  else {
    camera = camera_manager.cameras().front();
    logInfo("Using default camera: " + camera->id());
  }

  try {
    if (camera->acquire()) {
      logError("Failed to acquire camera");
      return false;
    }
  }
  catch (const std::exception &e) {
    logError("Exception during camera acquisition: " + std::string(e.what()));
    return false;
  }
  catch (...) {
    logError("Unknown exception during camera acquisition");
    return false;
  }

  // Configure the camera
  logInfo("Configuring camera...");
  if (!configureCamera(width, height)) {
    logError("Failed to configure camera");
    camera->release();
    camera.reset();
    return false;
  }

  // Create a clean camera name for info
  const auto &props = camera->properties();
  camera_name = camera->id() + '_' + stream->configuration().size.toString();
  if (auto model = props.get(libcamera::properties::Model)) {
    camera_name = model.value() + '_' + camera_name;
  }

  // Clean camera name
  for (auto &c : camera_name) {
    if (!std::isalnum(c))
      c = '_';
  }

  logInfo("Camera initialized successfully with name: " + camera_name);
  return true;
}

bool
WbCameraHandler::configureCamera(uint32_t width, uint32_t height)
{
  const libcamera::Size size {width, height};

  logInfo("Configuring camera with width=" + std::to_string(size.width) +
          ", height=" + std::to_string(size.height) + ", format=RGB888");

  try {
    std::unique_ptr<libcamera::CameraConfiguration> cfg =
      camera->generateConfiguration({libcamera::StreamRole::Viewfinder});

    if (!cfg) {
      logError("Failed to generate camera configuration");
      return false;
    }

    libcamera::StreamConfiguration &scfg = cfg->at(0);
    logInfo("Default configuration: " + scfg.toString());

    // Set our desired format
    scfg.pixelFormat = libcamera::formats::RGB888;
    scfg.size = size;

    // Validate configuration
    auto validation_status = cfg->validate();
    if (validation_status == libcamera::CameraConfiguration::Invalid) {
      logError("Invalid camera configuration");
      return false;
    }
    else if (validation_status == libcamera::CameraConfiguration::Adjusted) {
      logWarn("Stream configuration adjusted to: " + scfg.toString());
    }

    if (camera->configure(cfg.get()) < 0) {
      logError("Failed to apply camera configuration");
      return false;
    }

    logInfo("Camera successfully configured with " + scfg.toString() + " stream");

    // Get the configured stream
    stream = scfg.stream();

    // Allocate buffers
    logInfo("Allocating frame buffers...");
    allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
    allocator->allocate(stream);

    // Create requests
    logInfo("Creating camera requests...");
    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
      std::unique_ptr<libcamera::Request> request = camera->createRequest();
      if (!request) {
        logError("Failed to create request");
        return false;
      }

      // Map buffer memory
      size_t buffer_length = 0;
      int fd = -1;
      for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
        buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
        if (fd == -1) {
          fd = plane.fd.get();
        }
      }

      void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);
      if (data == MAP_FAILED) {
        logError("mmap failed: " + std::string(strerror(errno)));
        return false;
      }
      buffer_info[buffer.get()] = {data, buffer_length};

      if (request->addBuffer(stream, buffer.get()) < 0) {
        logError("Failed to set buffer for request");
        return false;
      }

      requests.push_back(std::move(request));
    }

    logInfo("Created " + std::to_string(requests.size()) + " camera requests");

    // Register callback
    camera->requestCompleted.connect(this, &WbCameraHandler::requestComplete);

    return true;
  }
  catch (const std::exception &e) {
    logError("Exception during camera configuration: " + std::string(e.what()));
    return false;
  }
  catch (...) {
    logError("Unknown exception during camera configuration");
    return false;
  }
}

std::string
WbCameraHandler::getCameraName() const
{
  return camera_name;
}

bool
WbCameraHandler::start(FrameCallback callback)
{
  if (running) {
    logError("Camera is already running");
    return false;
  }

  if (!camera || !stream) {
    logError("Camera is not initialized");
    return false;
  }

  frame_callback = callback;

  // Start camera with initial controls
  libcamera::ControlList controls;
  controls.set(libcamera::controls::AeEnable, auto_exposure_enabled);
  controls.set(libcamera::controls::ExposureTime, exposure_time_us);

  logInfo("Setting camera exposure: AE_ENABLE=" + std::string(auto_exposure_enabled ? "true" : "false") +
          ", EXPOSURE_TIME_US=" + std::to_string(exposure_time_us));

  logInfo("Starting camera capture...");
  try {
    if (camera->start(&controls)) {
      logError("Failed to start camera");
      return false;
    }
  }
  catch (const std::exception &e) {
    logError("Exception while starting camera: " + std::string(e.what()));
    return false;
  }
  catch (...) {
    logError("Unknown exception while starting camera");
    return false;
  }

  logInfo("Camera started successfully");
  running = true;

  // Queue all requests
  logInfo("Queueing initial camera requests...");
  try {
    for (std::unique_ptr<libcamera::Request> &request : requests) {
      if (camera->queueRequest(request.get()) < 0) {
        logError("Failed to queue request");
        camera->stop();
        running = false;
        return false;
      }
    }
  }
  catch (const std::exception &e) {
    logError("Exception while queueing requests: " + std::string(e.what()));
    camera->stop();
    running = false;
    return false;
  }
  catch (...) {
    logError("Unknown exception while queueing requests");
    camera->stop();
    running = false;
    return false;
  }

  logInfo("All camera requests queued successfully");
  return true;
}

void
WbCameraHandler::stop()
{
  if (!running) {
    logInfo("Camera is not running, nothing to stop");
    return;
  }

  logInfo("Stopping camera...");
  running = false;

  try {
    if (camera) {
      camera->stop();
      logInfo("Camera stopped");

      // Clean up memory mapped buffers
      logInfo("Cleaning up memory mapped buffers...");
      for (const auto &e : buffer_info) {
        if (munmap(e.second.data, e.second.size) == -1) {
          logError("munmap failed: " + std::string(strerror(errno)));
        }
      }
      buffer_info.clear();

      allocator->free(stream);
      allocator.reset();
      stream = nullptr;

      camera->release();
      camera.reset();
      logInfo("Camera resources released");
    }

    camera_manager.stop();
    logInfo("Camera manager stopped");
  }
  catch (const std::exception &e) {
    logError("Exception during camera shutdown: " + std::string(e.what()));
  }
  catch (...) {
    logError("Unknown exception during camera shutdown");
  }
}

void
WbCameraHandler::setExposure(bool auto_exposure, uint32_t exposure_time_us)
{
  logInfo("Setting exposure: auto=" + std::string(auto_exposure ? "true" : "false") +
          ", time=" + std::to_string(exposure_time_us) + " μs");
  auto_exposure_enabled = auto_exposure;
  this->exposure_time_us = exposure_time_us;

  if (running && camera) {
    try {
      // Create new control list
      libcamera::ControlList controls(camera->controls());

      controls.set(libcamera::controls::AeEnable, auto_exposure_enabled);
      controls.set(libcamera::controls::ExposureTime, this->exposure_time_us);

      // Create and queue a new request with updated controls
      auto request = camera->createRequest();
      if (!request) {
        logError("Failed to create exposure control request");
        return;
      }

      if (request->addBuffer(stream, allocator->buffers(stream).front().get()) < 0) {
        logError("Failed to add buffer to request");
        return;
      }

      request->controls() = std::move(controls);

      if (camera->queueRequest(request.get()) < 0) {
        logError("Failed to queue exposure update request");
      }
      else {
        logInfo("Exposure settings queued successfully");
      }
    }
    catch (const std::exception &e) {
      logError("Exception updating exposure: " + std::string(e.what()));
    }
    catch (...) {
      logError("Unknown exception during exposure update");
    }
  }
}

bool
WbCameraHandler::setupHardwareSync(const std::string &timeshare_path)
{
  this->timeshare_path = timeshare_path;
  logInfo("Setting up timeshare at path: " + timeshare_path);

  // Clean up previous mapping if it exists
  if (pointt != (time_stamp *)MAP_FAILED) {
    if (munmap(pointt, sizeof(time_stamp)) == -1) {
      logError("Failed to unmap previous timeshare: " + std::string(strerror(errno)));
    }
    pointt = (time_stamp *)MAP_FAILED;
  }

  // Try to open the timeshare file
  int fd = open(timeshare_path.c_str(), O_RDWR);
  if (fd < 0) {
    logError("Could not open timeshare file " + timeshare_path + ": " + std::string(strerror(errno)));
    logInfo("Will use system time for timestamps");
    return false;
  }

  // Check if file exists and is appropriate size
  struct stat file_stat;
  if (fstat(fd, &file_stat) < 0) {
    logWarn("Could not get timeshare file stats: " + std::string(strerror(errno)));
  }
  else {
    logInfo("Timeshare file size: " + std::to_string(file_stat.st_size) + " bytes");

    // The file is likely a sparse file with holes. For sparse files, st_size might report
    // a smaller size than what's needed for mapping the full time_stamp structure.
    // We'll accept as long as there's enough space for the "low" timestamp (8 bytes)
    if (file_stat.st_size < 16) { // Minimum size needed for high and low values
      logError("Timeshare file size too small: " + std::to_string(file_stat.st_size) +
               " bytes (expected at least 16 bytes)");
      close(fd);
      return false;
    }
  }

  // Map the shared memory
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);
  close(fd); // File descriptor can be closed after mapping

  if (pointt == (time_stamp *)MAP_FAILED) {
    logError("mmap failed for timeshare file: " + std::string(strerror(errno)));
    return false;
  }

  // Output initial timestamp values
  logInfo("Successfully mapped timeshare file for hardware synchronization");
  logInfo("Initial timeshare values - high: " + std::to_string(pointt->high) +
          ", low: " + std::to_string(pointt->low));

  // Convert the LiDAR timestamp to a human-readable date for verification
  if (pointt->low != 0) {
    time_t seconds = pointt->low / 1000000000;
    char time_buf[32];
    struct tm tm_info;

    // Convert to local time
    localtime_r(&seconds, &tm_info);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);

    std::stringstream timestamp_ss;
    timestamp_ss << time_buf << "." << std::setfill('0') << std::setw(9) << (pointt->low % 1000000000);
    logInfo("LiDAR timestamp in human-readable form: " + timestamp_ss.str());

    // Compare to system time
    auto current_time = std::chrono::system_clock::now();
    auto duration = current_time.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    seconds = nanoseconds / 1000000000;
    localtime_r(&seconds, &tm_info);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);

    timestamp_ss.str("");
    timestamp_ss << time_buf << "." << std::setfill('0') << std::setw(9) << (nanoseconds % 1000000000);
    logInfo("System time in human-readable form: " + timestamp_ss.str());

    double diff_seconds = std::abs(double(pointt->low) / 1.0e9 - double(nanoseconds) / 1.0e9);
    logInfo("Time difference between LiDAR and system: " + std::to_string(diff_seconds) + " seconds");

    if (diff_seconds > 3600) {
      logWarn("LiDAR and system times differ by more than an hour!");
      logWarn("This may indicate different time epochs are being used.");
    }
  }
  else {
    logWarn("Initial LiDAR timestamp is zero. Waiting for LiDAR to update timestamp...");
  }

  return true;
}

void
WbCameraHandler::requestComplete(libcamera::Request *request)
{
  if (!running) {
    logDebug("Request completed but camera is not running, ignoring");
    return;
  }

  if (request->status() == libcamera::Request::RequestComplete) {
    try {
      // Get buffer and metadata
      const libcamera::FrameBuffer *buffer = request->findBuffer(stream);
      if (!buffer) {
        logError("Completed request has no buffer for our stream");
        return;
      }

      const libcamera::FrameMetadata &metadata = buffer->metadata();

      // Get camera timestamp from metadata (in ns)
      int64_t camera_timestamp_ns = 0;
      for (const auto &plane : metadata.planes()) {
        if (plane.bytesused > 0) {
          camera_timestamp_ns = metadata.timestamp;
          break;
        }
      }

      // Hardware timestamp from shared memory
      int64_t hw_timestamp_ns = 0;
      bool hw_timestamp_valid = false;

      if (pointt != (time_stamp *)MAP_FAILED) {
        hw_timestamp_ns = pointt->low;
        hw_timestamp_valid = (hw_timestamp_ns != 0);
      }

      // Prepare frame data to pass to callback
      const libcamera::StreamConfiguration &cfg = stream->configuration();
      FrameData frame_data;
      frame_data.data = static_cast<const uint8_t *>(buffer_info[buffer].data);
      frame_data.size = buffer_info[buffer].size;
      frame_data.width = cfg.size.width;
      frame_data.height = cfg.size.height;
      frame_data.stride = cfg.stride;
      frame_data.camera_timestamp_ns = camera_timestamp_ns;
      frame_data.hw_timestamp_ns = hw_timestamp_ns;
      frame_data.hw_timestamp_valid = hw_timestamp_valid;

      // Call the callback with the frame data
      if (frame_callback) {
        frame_callback(frame_data);
      }

      // Queue the request again with the same control values
      request->reuse(libcamera::Request::ReuseBuffers);
      request->controls().set(libcamera::controls::AeEnable, auto_exposure_enabled);
      request->controls().set(libcamera::controls::ExposureTime, exposure_time_us);

      if (camera->queueRequest(request) < 0) {
        logError("Failed to requeue request");
      }
    }
    catch (const std::exception &e) {
      logError("Exception in requestComplete: " + std::string(e.what()));
    }
    catch (...) {
      logError("Unknown exception in requestComplete");
    }
  }
  else {
    logWarn("Request returned with status: " + std::to_string((int)request->status()));

    // Try to requeue the request anyway
    try {
      request->reuse(libcamera::Request::ReuseBuffers);
      request->controls().set(libcamera::controls::AeEnable, auto_exposure_enabled);
      request->controls().set(libcamera::controls::ExposureTime, exposure_time_us);
      camera->queueRequest(request);
    }
    catch (const std::exception &e) {
      logError("Failed to requeue failed request: " + std::string(e.what()));
    }
  }
}

// Flip RGB888 image vertically and horizontally
void
WbCameraHandler::flipRGB888Image(uint8_t *data, int width, int height, int step)
{
  // Create a temporary buffer for a single row
  std::vector<uint8_t> temp_row(step);

  // Flip vertically (swap rows from top and bottom)
  for (int y = 0; y < height / 2; y++) {
    uint8_t *top_row = data + y * step;
    uint8_t *bottom_row = data + (height - 1 - y) * step;

    // Swap entire rows
    memcpy(temp_row.data(), top_row, step);
    memcpy(top_row, bottom_row, step);
    memcpy(bottom_row, temp_row.data(), step);
  }

  // Flip horizontally (RGB pixels within each row)
  for (int y = 0; y < height; y++) {
    uint8_t *row = data + y * step;
    for (int x = 0; x < width / 2; x++) {
      // Swap RGB values (3 bytes per pixel)
      std::swap(row[x * 3], row[(width - 1 - x) * 3]);         // R
      std::swap(row[x * 3 + 1], row[(width - 1 - x) * 3 + 1]); // G
      std::swap(row[x * 3 + 2], row[(width - 1 - x) * 3 + 2]); // B
    }
  }
}

} // namespace camera
