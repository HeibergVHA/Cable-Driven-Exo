#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 
#include "std_msgs/msg/int32_multi_array.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

// Inspiration drawn from https://github.com/Seeed-Studio/Seeed_Arduino_AS5600/blob/master/src/AS5600.cpp

class AS5600Reader : public rclcpp::Node
{
public:
    AS5600Reader() : Node("as5600_reader")
    {
        declare_parameter<double>("publish_rate_hz", 200.0);       // Read frequency
        declare_parameter<std::vector<int64_t>>("channels", std::vector<int64_t>{2, 3});
        declare_parameter<bool>("status_out", false);

        const double rate_hz = get_parameter("publish_rate_hz").as_double();
        const auto raw_channels = get_parameter("channels").as_integer_array();
        status_out_ = get_parameter("status_out").as_bool();

        // validate parameters
        if (rate_hz <= 0.0) {
            throw std::runtime_error("publish_rate_hz must be > 0");
        }
        if (raw_channels.empty()) {
            throw std::runtime_error("channels must not be empty");
        }
        for (const auto ch : raw_channels) {
            if (ch < 0 || ch > 7) {
                throw std::runtime_error("each channel must be in the range 0-7");
            }
            channels_.push_back(static_cast<uint8_t>(ch));
        }

        joint_state_.name.resize(channels_.size());
        joint_state_.position.resize(channels_.size());
        for (size_t i = 0; i < channels_.size(); ++i) {
            joint_state_.name[i] = "joint_" + std::to_string(channels_[i]);
        }
        joint_sensor_status_.data.resize(channels_.size());
        
        joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        status_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>("joint_sensor_status", 10);


        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate_hz));   // 1/rate = seconds
        timer_ = create_wall_timer(period, [this] { read_sensors(); });

        fd_ = open("/dev/i2c-1", O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open /dev/i2c-1");
        }

        RCLCPP_INFO(get_logger(), "AS5600 reader started");
    }

    ~AS5600Reader() override
    {
        if (fd_ >= 0) {
            try { select_mux_channel(0x00); } catch (...) {}
            close(fd_);
        }
    }

private:
    static constexpr uint8_t kMuxAddr    = 0x70;
    static constexpr uint8_t kSensorAddr = 0x36;
    static constexpr uint8_t kAngleReg   = 0x0E; // AS5600 12-Bit Programmable Contactless Potentiometer: page 18
    static constexpr uint8_t kStatusReg   = 0x0B; // https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf

    int fd_{-1};
    bool status_out_{false};
    std::vector<uint8_t> channels_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
    std_msgs::msg::Int32MultiArray joint_sensor_status_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr status_publisher_;

    void set_address(uint8_t addr)
    {
        if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
            throw std::runtime_error("Failed to set I2C address");
        }
    }

    void select_mux_channel(uint8_t mask)
    {
        set_address(kMuxAddr);
        if (write(fd_, &mask, 1) != 1) {
            throw std::runtime_error("Failed to write multiplexer channel");
        }
    }

    void select_sensor(uint8_t channel)
    {
        select_mux_channel(static_cast<uint8_t>(1U << channel)); // 1U = 00000001. Example Channel = 3: 00000001 << 3 = 00001000, which is what the multiplexer wants to select channel 3.
        set_address(kSensorAddr);
    }

    double read_angle_deg(uint8_t channel)
    {
        select_sensor(channel);

        uint8_t reg = kAngleReg;
        uint8_t data[2];
        if (write(fd_, &reg, 1) != 1 || read(fd_, data, 2) != 2) {
            throw std::runtime_error("Failed to read AS5600 angle");
        }

        const uint16_t raw = ((data[0] & 0x0F) << 8) | data[1];
        return raw * 360.0 / 4096.0;
    }

    int read_status(uint8_t channel)
    {
        select_sensor(channel);

        int retVal = 0;
        uint8_t reg = kStatusReg;
        uint8_t magStatus;

        if (write(fd_, &reg, 1) != 1 || read(fd_, &magStatus, 1) != 1) {
            throw std::runtime_error("Failed to read AS5600 status");
        }

        if (magStatus & 0x20) {
            retVal = 2;   // Magnet detected
            if (magStatus & 0x10)
                retVal = 1; // Magnet too weak
            else if (magStatus & 0x08)
                retVal = 3; // Magnet too strong
        }
        return retVal;
    }

    void read_sensors()
    {
        try {
            // for (const auto ch : channels_) {
            //     RCLCPP_INFO(get_logger(), "CH %u: %.2f deg", static_cast<unsigned>(ch), read_angle_deg(ch));
            // }
            for (size_t i = 0; i < channels_.size(); ++i) {
                joint_state_.position[i] = read_angle_deg(channels_[i]);
                if (status_out_)
                    joint_sensor_status_.data[i] = read_status(channels_[i]);
            }
            joint_state_.header.stamp = now();
            joint_publisher_->publish(joint_state_);
            
            if (status_out_){
                //joint_sensor_status_.header.stamp = now();
                status_publisher_->publish(joint_sensor_status_);
            }

        } catch (const std::exception &e) {
            // Throttled error messages
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 200, "%s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<AS5600Reader>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("as5600_reader"), "%s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}