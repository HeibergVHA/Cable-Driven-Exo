#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp> 

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

class AS5600Reader : public rclcpp::Node
{
public:
    AS5600Reader() : Node("as5600_reader")
    {
        declare_parameter<double>("publish_rate_hz", 200.0);       // Read frequency
        declare_parameter<std::vector<int64_t>>("channels", std::vector<int64_t>{2, 3});

        const double rate_hz = get_parameter("publish_rate_hz").as_double();
        const auto raw_channels = get_parameter("channels").as_integer_array();

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

        publisher_ = create_publisher<sensor_msgs::msg::JointState>(topic, 10);



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
            try { select_channel(0x00); } catch (...) {}
            close(fd_);
        }
    }

private:
    static constexpr uint8_t kMuxAddr    = 0x70;
    static constexpr uint8_t kSensorAddr = 0x36;
    static constexpr uint8_t kAngleReg   = 0x0E;

    int fd_{-1};
    std::vector<uint8_t> channels_;          // FIX: member instead of a local
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    void set_address(uint8_t addr)
    {
        if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
            throw std::runtime_error("Failed to set I2C address");
        }
    }

    void select_channel(uint8_t mask)
    {
        set_address(kMuxAddr);
        if (write(fd_, &mask, 1) != 1) {
            throw std::runtime_error("Failed to write multiplexer channel");
        }
    }

    double read_angle_deg(uint8_t channel)
    {
        select_channel(static_cast<uint8_t>(1U << channel));
        set_address(kSensorAddr);

        uint8_t reg = kAngleReg;
        uint8_t data[2];
        if (write(fd_, &reg, 1) != 1 || read(fd_, data, 2) != 2) {
            throw std::runtime_error("Failed to read AS5600 angle");
        }

        const uint16_t raw = ((data[0] & 0x0F) << 8) | data[1];
        return raw * 360.0 / 4096.0;
    }

    void read_sensors()
    {
        try {
            // for (const auto ch : channels_) {
            //     RCLCPP_INFO(get_logger(), "CH %u: %.2f deg", static_cast<unsigned>(ch), read_angle_deg(ch));
            // }
            for (const auto ch : channels_) {
                joint_state_.position[i] = read_angle_deg(ch);
            }
            joint_state_.header.stamp = now();
            publisher_->publish(joint_state_);

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