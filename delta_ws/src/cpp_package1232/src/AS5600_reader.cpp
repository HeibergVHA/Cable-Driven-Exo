#include <rclcpp/rclcpp.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <stdexcept>

class AS5600Reader : public rclcpp::Node
{
public:
    AS5600Reader() : Node("as5600_reader")
    {
        // Configuration
        bus_number_ = 1;
        mux_address_ = 0x70;
        sensor_address_ = 0x36;

        channel_1_ = 2;
        channel_2_ = 3;

        // Open I2C bus
        const std::string device =
            "/dev/i2c-" + std::to_string(bus_number_);

        i2c_fd_ = open(device.c_str(), O_RDWR);

        if (i2c_fd_ < 0) {
            throw std::runtime_error("Failed to open " + device);
        }

        RCLCPP_INFO(get_logger(), "AS5600 reader started");

        // Read continuously at 20 Hz
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AS5600Reader::read_sensors, this));
    }

    ~AS5600Reader() override
    {
        if (i2c_fd_ >= 0) {
            // Disable all multiplexer channels
            try {
                select_mux_channel(0x00);
            } catch (...) {
                // Ignore errors during shutdown
            }

            close(i2c_fd_);
        }
    }

private:
    int i2c_fd_{-1};

    int bus_number_;
    uint8_t mux_address_;
    uint8_t sensor_address_;

    uint8_t channel_1_;
    uint8_t channel_2_;

    rclcpp::TimerBase::SharedPtr timer_;

    void select_mux_channel(uint8_t mask)
    {
        if (ioctl(i2c_fd_, I2C_SLAVE, mux_address_) < 0) {
            throw std::runtime_error("Failed to select I2C multiplexer");
        }

        if (write(i2c_fd_, &mask, 1) != 1) {
            throw std::runtime_error("Failed to write multiplexer channel");
        }
    }

    uint16_t read_raw_angle(uint8_t channel)
    {
        // Select the multiplexer channel
        select_mux_channel(1U << channel);

        // Select AS5600 ANGLE register 0x0E
        if (ioctl(i2c_fd_, I2C_SLAVE, sensor_address_) < 0) {
            throw std::runtime_error("Failed to select AS5600 address");
        }

        uint8_t reg = 0x0E;

        if (write(i2c_fd_, &reg, 1) != 1) {
            throw std::runtime_error("Failed to select angle register");
        }

        // Read MSB and LSB
        uint8_t data[2];

        if (read(i2c_fd_, data, 2) != 2) {
            throw std::runtime_error("Failed to read AS5600 angle");
        }

        // AS5600 angle is a 12-bit value
        return (static_cast<uint16_t>(data[0] & 0x0F) << 8)
               | data[1];
    }

    void read_sensors()
    {
        try {
            const uint16_t raw1 = read_raw_angle(channel_1_);
            const uint16_t raw2 = read_raw_angle(channel_2_);

            const double angle1 = raw1 * 360.0 / 4096.0;
            const double angle2 = raw2 * 360.0 / 4096.0;

            RCLCPP_INFO(
                get_logger(),
                "Sensor 1 (CH %u): %.2f deg | Sensor 2 (CH %u): %.2f deg",
                channel_1_, angle1,
                channel_2_, angle2);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<AS5600Reader>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("as5600_reader"),
            "%s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
