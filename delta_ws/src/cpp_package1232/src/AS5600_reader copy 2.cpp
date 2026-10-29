#include <rclcpp/rclcpp.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

class AS5600Reader : public rclcpp::Node
{
public:
    AS5600Reader() : Node("as5600_reader")
    {
        fd_ = open("/dev/i2c-1", O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open /dev/i2c-1");
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50), [this] { read_sensors(); });

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
    static constexpr std::array<uint8_t, 2> kChannels{2, 3};

    int fd_{-1};
    rclcpp::TimerBase::SharedPtr timer_;

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
        select_channel(1U << channel);
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
            for (auto ch : kChannels) {
                RCLCPP_INFO(get_logger(), "CH %u: %.2f deg", ch, read_angle_deg(ch));
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<AS5600Reader>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("as5600_reader"), "%s", e.what());
    }
    rclcpp::shutdown();
}