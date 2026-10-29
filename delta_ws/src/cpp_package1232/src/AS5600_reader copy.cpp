// ============================================================================
// as5600_publisher.cpp
//
// ROS 2 node that reads several AS5600 magnetic encoders through a
// TCA9548A / PCA9548A I2C multiplexer and publishes their angles as a
// sensor_msgs/msg/JointState on the "joint_states" topic.
//
// Improvements over the original version:
//   * Publishes to a topic instead of only logging
//   * Everything configurable through ROS parameters (no recompiling)
//   * Uses I2C_RDWR combined transfers (repeated start, no I2C_SLAVE juggling)
//   * Skips redundant multiplexer switching (remembers the active channel)
//   * Detects missing magnets via the AS5600 STATUS register
//   * Optional multi-turn unwrapping, direction flip and zero offset
//   * Throttled error logging so a broken sensor doesn't flood the console
//   * No file-descriptor leak if the constructor fails
// ============================================================================

// --- ROS 2 headers ----------------------------------------------------------
#include <rclcpp/rclcpp.hpp>                       // Node, timers, logging, parameters
#include <sensor_msgs/msg/joint_state.hpp>         // Message type we publish

// --- Linux I2C headers ------------------------------------------------------
#include <linux/i2c.h>                             // struct i2c_msg, I2C_M_RD
#include <linux/i2c-dev.h>                         // struct i2c_rdwr_ioctl_data, I2C_RDWR
#include <sys/ioctl.h>                             // ioctl()
#include <fcntl.h>                                 // open(), O_RDWR
#include <unistd.h>                                // close()

// --- C++ standard library ---------------------------------------------------
#include <cerrno>                                  // errno
#include <chrono>                                  // timer period types
#include <cmath>                                   // M_PI
#include <cstdint>                                 // fixed-width integer types
#include <cstring>                                 // std::strerror
#include <memory>                                  // std::make_shared
#include <stdexcept>                               // std::runtime_error
#include <string>                                  // std::string
#include <vector>                                  // std::vector

// ----------------------------------------------------------------------------
// AS5600 constants (see the AS5600 datasheet, register map)
// ----------------------------------------------------------------------------
namespace as5600
{
constexpr uint8_t kStatusReg    = 0x0B;   // STATUS register: magnet detection flags
constexpr uint8_t kAngleReg     = 0x0E;   // ANGLE register (2 bytes, filtered, 12 bit)
constexpr uint8_t kMagnetDetect = 0x20;   // STATUS bit 5 (MD): magnet detected
constexpr int     kCountsPerRev = 4096;   // 12-bit resolution
constexpr int     kAngleMask    = 0x0FFF; // keep only the lower 12 bits
}  // namespace as5600

// ----------------------------------------------------------------------------
// Per-sensor configuration and runtime state
// ----------------------------------------------------------------------------
struct SensorState
{
  // ---- Configuration (loaded once from parameters) ----
  uint8_t     mux_channel{0};   // Multiplexer channel (0-7) this sensor sits on
  std::string joint_name;       // Name used in the JointState message
  double      direction{1.0};   // +1.0 or -1.0 to flip rotation direction
  double      offset_rad{0.0};  // Zero offset added to the final angle [rad]

  // ---- Runtime state (used for multi-turn unwrapping) ----
  bool    initialized{false};   // True after the first successful reading
  int     last_raw{0};          // Previous raw reading (0-4095)
  int64_t continuous{0};        // Accumulated counts across full turns
};

// ----------------------------------------------------------------------------
// The node
// ----------------------------------------------------------------------------
class AS5600Publisher : public rclcpp::Node
{
public:
  AS5600Publisher()
  : Node("as5600_publisher")
  {
    // ---- Step 1: declare parameters with sensible defaults ----------------
    // Declaring parameters makes them visible/overridable from the command
    // line, launch files and YAML files.
    declare_parameter<int>("bus_number", 1);                  // /dev/i2c-<N>
    declare_parameter<int>("mux_address", 0x70);              // TCA9548A address
    declare_parameter<int>("sensor_address", 0x36);           // AS5600 address (fixed)
    declare_parameter<double>("publish_rate_hz", 20.0);       // Read/publish frequency
    declare_parameter<std::string>("topic", "joint_states");  // Output topic name
    declare_parameter<std::string>("frame_id", "");           // Header frame id
    declare_parameter<bool>("unwrap", true);                  // Multi-turn tracking on/off

    declare_parameter<std::vector<int64_t>>(
      "mux_channels", std::vector<int64_t>{2, 3});            // One entry per sensor
    declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint_1", "joint_2"});
    declare_parameter<std::vector<double>>(
      "directions", std::vector<double>{1.0, 1.0});           // +1 or -1 per sensor
    declare_parameter<std::vector<double>>(
      "offsets_rad", std::vector<double>{0.0, 0.0});          // Zero offset per sensor

    // ---- Step 2: read the parameter values --------------------------------
    const int bus_number = get_parameter("bus_number").as_int();
    mux_address_    = static_cast<uint8_t>(get_parameter("mux_address").as_int());
    sensor_address_ = static_cast<uint8_t>(get_parameter("sensor_address").as_int());
    unwrap_         = get_parameter("unwrap").as_bool();

    const double rate_hz        = get_parameter("publish_rate_hz").as_double();
    const std::string topic     = get_parameter("topic").as_string();
    frame_id_                   = get_parameter("frame_id").as_string();

    const auto channels   = get_parameter("mux_channels").as_integer_array();
    const auto names      = get_parameter("joint_names").as_string_array();
    const auto directions = get_parameter("directions").as_double_array();
    const auto offsets    = get_parameter("offsets_rad").as_double_array();

    // ---- Step 3: validate the parameters ----------------------------------
    // Fail early with a clear message instead of misbehaving later.
    if (rate_hz <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be > 0");
    }
    if (channels.empty()) {
      throw std::runtime_error("mux_channels must contain at least one channel");
    }
    if (names.size() != channels.size() ||
      directions.size() != channels.size() ||
      offsets.size() != channels.size()) {
      throw std::runtime_error(
              "mux_channels, joint_names, directions and offsets_rad "
              "must all have the same length");
    }

    // ---- Step 4: build the per-sensor table -------------------------------
    sensors_.resize(channels.size());
    for (size_t i = 0; i < channels.size(); ++i) {
      if (channels[i] < 0 || channels[i] > 7) {
        throw std::runtime_error("mux channel must be in the range 0-7");
      }
      sensors_[i].mux_channel = static_cast<uint8_t>(channels[i]);
      sensors_[i].joint_name  = names[i];
      sensors_[i].direction   = (directions[i] < 0.0) ? -1.0 : 1.0;
      sensors_[i].offset_rad  = offsets[i];
    }

    // ---- Step 5: prepare the reusable JointState message ------------------
    // Names never change, so fill them once. Positions are updated each cycle.
    joint_state_.name.reserve(sensors_.size());
    for (const auto & s : sensors_) {
      joint_state_.name.push_back(s.joint_name);
    }
    joint_state_.position.resize(sensors_.size(), 0.0);
    joint_state_.header.frame_id = frame_id_;

    // ---- Step 6: create the publisher -------------------------------------
    // Queue depth 10 is the standard for joint_states.
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    // ---- Step 7: open the I2C bus -----------------------------------------
    // This is done LAST among the steps that can throw. If an exception were
    // thrown after open(), the destructor would not run and the file
    // descriptor would leak.
    const std::string device = "/dev/i2c-" + std::to_string(bus_number);
    i2c_fd_ = open(device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      throw std::runtime_error(
              "Failed to open " + device + ": " + std::strerror(errno));
    }

    // ---- Step 8: start the periodic timer ---------------------------------
    // Convert the rate in Hz to a period; the timer then calls update().
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() {update();});

    RCLCPP_INFO(
      get_logger(), "AS5600 publisher started: %zu sensor(s) on %s -> '%s' at %.1f Hz",
      sensors_.size(), device.c_str(), topic.c_str(), rate_hz);
  }

  // --------------------------------------------------------------------------
  // Destructor: leave the bus in a clean state
  // --------------------------------------------------------------------------
  ~AS5600Publisher() override
  {
    if (i2c_fd_ >= 0) {
      // Disconnect every downstream multiplexer channel.
      try {
        select_mux_mask(0x00);
      } catch (...) {
        // Nothing sensible to do about errors while shutting down.
      }
      // Release the file descriptor.
      close(i2c_fd_);
      i2c_fd_ = -1;
    }
  }

private:
  // ==========================================================================
  // Low-level I2C helpers
  // ==========================================================================

  // Perform one I2C_RDWR ioctl: a sequence of messages executed as a single
  // transaction (with repeated START between messages, no STOP in between).
  void i2c_transfer(i2c_msg * msgs, uint32_t count)
  {
    i2c_rdwr_ioctl_data data{};   // Zero-initialise the request structure
    data.msgs  = msgs;            // Pointer to the array of messages
    data.nmsgs = count;           // Number of messages in the array

    if (ioctl(i2c_fd_, I2C_RDWR, &data) < 0) {
      throw std::runtime_error(
              std::string("I2C transfer failed: ") + std::strerror(errno));
    }
  }

  // Write a bitmask to the multiplexer's control register.
  // Bit N set => downstream channel N is connected to the main bus.
  void select_mux_mask(uint8_t mask)
  {
    // Nothing to do if this channel set is already active.
    if (current_mask_ == mask) {
      return;
    }

    // Mark the state as unknown BEFORE the transfer: if it fails halfway we
    // don't know what the mux is doing, so the next call must resend.
    current_mask_ = -1;

    // Single write message: [mux address] [mask]
    i2c_msg msg{};
    msg.addr  = mux_address_;
    msg.flags = 0;                // 0 = write
    msg.len   = 1;
    msg.buf   = &mask;

    i2c_transfer(&msg, 1);

    // Success: remember what is now active.
    current_mask_ = mask;
  }

  // Read `length` bytes starting at register `reg` of the AS5600.
  // Uses a combined write-then-read transaction (repeated START).
  void read_sensor_register(uint8_t reg, uint8_t * buffer, uint16_t length)
  {
    i2c_msg msgs[2]{};

    // Message 1: write the register address (sets the sensor's register pointer).
    msgs[0].addr  = sensor_address_;
    msgs[0].flags = 0;            // write
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    // Message 2: read `length` bytes back from that register onward.
    msgs[1].addr  = sensor_address_;
    msgs[1].flags = I2C_M_RD;     // read
    msgs[1].len   = length;
    msgs[1].buf   = buffer;

    i2c_transfer(msgs, 2);
  }

  // ==========================================================================
  // Sensor-level helpers
  // ==========================================================================

  // Read the raw 12-bit angle (0-4095) of the sensor on a given mux channel.
  uint16_t read_raw_angle(uint8_t channel)
  {
    // Connect only this sensor's channel to the main bus (bit shift = bitmask).
    select_mux_mask(static_cast<uint8_t>(1U << channel));

    // Check the STATUS register to make sure a magnet is detected;
    // otherwise the angle value is meaningless.
    uint8_t status = 0;
    read_sensor_register(as5600::kStatusReg, &status, 1);
    if ((status & as5600::kMagnetDetect) == 0) {
      throw std::runtime_error(
              "magnet not detected on mux channel " + std::to_string(channel));
    }

    // Read the two ANGLE bytes: data[0] = high byte, data[1] = low byte.
    uint8_t data[2] = {0, 0};
    read_sensor_register(as5600::kAngleReg, data, 2);

    // Combine into a 12-bit value (upper 4 bits of the high byte are unused).
    const uint16_t raw =
      static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return raw & as5600::kAngleMask;
  }

  // Convert a fresh raw reading into the final angle in radians,
  // applying unwrapping, direction and offset.
  double raw_to_radians(SensorState & s, int raw)
  {
    constexpr double kRadPerCount = 2.0 * M_PI / as5600::kCountsPerRev;

    double counts = static_cast<double>(raw);   // Default: absolute angle 0..4095

    if (unwrap_) {
      if (!s.initialized) {
        // First sample: start counting from here.
        s.continuous  = raw;
        s.initialized = true;
      } else {
        // Difference to the previous reading.
        int delta = raw - s.last_raw;

        // If the jump is larger than half a turn, the sensor wrapped around
        // 0/4095. Correct by one full revolution in the opposite direction.
        if (delta > as5600::kCountsPerRev / 2) {
          delta -= as5600::kCountsPerRev;
        } else if (delta < -as5600::kCountsPerRev / 2) {
          delta += as5600::kCountsPerRev;
        }
        s.continuous += delta;                  // Accumulate the true movement
      }
      s.last_raw = raw;                         // Remember for next time
      counts = static_cast<double>(s.continuous);
    }

    // Counts -> radians, then flip direction if configured, then add offset.
    return s.direction * counts * kRadPerCount + s.offset_rad;
  }

  // ==========================================================================
  // Timer callback: read all sensors and publish one message
  // ==========================================================================
  void update()
  {
    try {
      // ---- Step 1: read every sensor ----
      for (size_t i = 0; i < sensors_.size(); ++i) {
        const uint16_t raw = read_raw_angle(sensors_[i].mux_channel);
        joint_state_.position[i] = raw_to_radians(sensors_[i], raw);
      }

      // ---- Step 2: stamp the message with the current time ----
      joint_state_.header.stamp = now();

      // ---- Step 3: publish ----
      publisher_->publish(joint_state_);

    } catch (const std::exception & e) {
      // Any failure => skip this cycle (never publish partial/stale data).
      // Throttled to once every 2 s so a disconnected sensor won't flood logs.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Sensor read failed: %s", e.what());

      // The mux/sensor state is uncertain after an error: force a resend of
      // the channel selection on the next cycle.
      current_mask_ = -1;
    }
  }

  // ==========================================================================
  // Member variables
  // ==========================================================================
  int i2c_fd_{-1};                    // File descriptor of /dev/i2c-N (-1 = closed)
  uint8_t mux_address_{0x70};         // I2C address of the multiplexer
  uint8_t sensor_address_{0x36};      // I2C address of every AS5600
  int current_mask_{-1};              // Mux channels currently enabled (-1 = unknown)
  bool unwrap_{true};                 // Multi-turn tracking enabled?
  std::string frame_id_;              // Header frame id for the message

  std::vector<SensorState> sensors_;  // One entry per encoder
  sensor_msgs::msg::JointState joint_state_;  // Reused message buffer

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ----------------------------------------------------------------------------
// Entry point
// ----------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  // Initialise ROS 2 (parses ROS-specific command-line arguments).
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    // Create the node (may throw on bad parameters or missing I2C device).
    auto node = std::make_shared<AS5600Publisher>();

    // Process timer callbacks until Ctrl+C.
    rclcpp::spin(node);

  } catch (const std::exception & e) {
    // Report fatal startup/runtime errors and return a non-zero exit code.
    RCLCPP_FATAL(rclcpp::get_logger("as5600_publisher"), "%s", e.what());
    exit_code = 1;
  }

  // Cleanly shut down ROS 2.
  rclcpp::shutdown();
  return exit_code;
}