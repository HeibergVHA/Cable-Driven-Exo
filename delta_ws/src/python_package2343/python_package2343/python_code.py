from smbus2 import SMBus

TCAADDR = 0x70
BUS = 1

def tcaselect(bus, channel):
    if channel > 7:
        return
    bus.write_byte(TCAADDR, 1 << channel)

def tcadisable(bus):
    bus.write_byte(TCAADDR, 0x00)

with SMBus(BUS) as bus:
    print("TCAScanner ready!")

    for ch in range(8):
        tcaselect(bus, ch)
        print(f"TCA Port #{ch}")

        for addr in range(0x03, 0x78):  # valid 7-bit addresses
            if addr == TCAADDR:
                continue
            try:
                bus.read_byte(addr)
                print(f"  Found I2C 0x{addr:02X}")
            except OSError:
                pass  # no device at this address

    tcadisable(bus)  # leave all channels off
    print("done")