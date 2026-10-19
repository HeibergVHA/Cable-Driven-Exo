import rclpy
from rclpy.node import Node
from smbus2 import SMBus


class PythonNode(Node):
    TCAADDR = 0x70
    BUS = 1
    AS5600_ADDR = 0x36

    def __init__(self):
        super().__init__('tca_scanner')
        self.scan_sensors()

    def tcaselect(self, bus, channel):
        bus.write_byte(self.TCAADDR, 1 << channel)

    def tcadisable(self, bus):
        bus.write_byte(self.TCAADDR, 0x00)

    def read_angle(self, bus, addr=0x36):
        data = bus.read_i2c_block_data(addr, 0x0E, 2)

        raw = ((data[0] & 0x0F) << 8) | data[1]
        degrees = raw * 360.0 / 4096.0

        return raw, degrees

    def scan_sensors(self):
        with SMBus(self.BUS) as bus:
            try:
                for ch in range(8):
                    self.tcaselect(bus, ch)

                    try:
                        raw, degrees = self.read_angle(
                            bus, self.AS5600_ADDR
                        )

                        self.get_logger().info(
                            f"Channel {ch}: "
                            f"Raw={raw}, Angle={degrees:.2f} degrees"
                        )

                    except OSError:
                        self.get_logger().info(
                            f"Channel {ch}: No AS5600 detected"
                        )

            finally:
                self.tcadisable(bus)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PythonNode()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()