import rclpy
from rclpy.node import Node

from smbus2 import SMBus

class PythonNode(Node):

    TCAADDR = 0x70
    BUS = 1

    def __init__(self):
        super().__init__('tca_scanner')
        self.scan()

    def tcaselect(self, bus, channel):
        if channel > 7:
            return
        bus.write_byte(self.TCAADDR, 1 << channel)

    def tcadisable(self, bus):
        bus.write_byte(self.TCAADDR, 0x00)

    def scan(self):
        with SMBus(self.BUS) as bus:
            self.get_logger().info('TCAScanner')

            for ch in range(8):
                self.tcaselect(bus, ch)
                self.get_logger().info(f"TCA Port #{ch}")

                for addr in range(0x03, 0x78):  # valid 7-bit addresses
                    if addr == self.TCAADDR:
                        continue
                    try:
                        bus.read_byte(addr)
                        self.get_logger().info(f"  Found I2C 0x{addr:02X}")
                    except OSError:
                        # print(f"  No device found")
                        pass  # no device at this address

            self.tcadisable(bus)  # leave all channels off
            print("done")




def main(args=None):
    rclpy.init(args=args)
    node = PythonNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()