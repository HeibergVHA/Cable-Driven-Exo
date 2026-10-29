#!/bin/bash
# Bring up the INNO-MAKER USB2CAN adapter as can0 and install python-can.
# Usage: bash setup_can.sh [bitrate]     (default 1000000 = 1 Mbit/s)
# The bitrate MUST match the motors' CAN baud rate (125k / 250k / 500k / 1M).
set -e
BITRATE=${1:-1000000}

python3 -c "import can" 2>/dev/null || sudo apt-get install -y python3-can can-utils

if ! ip link show can0 >/dev/null 2>&1; then
    echo "can0 not found. Is the USB2CAN plugged in? (it uses the gs_usb kernel driver)"
    sudo modprobe gs_usb || true
    sleep 1
    ip link show can0 >/dev/null 2>&1 || { echo "Still no can0 - check 'dmesg | tail'"; exit 1; }
fi

sudo ip link set can0 down
sudo ip link set can0 type can bitrate "$BITRATE"
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
echo "can0 up at $BITRATE bit/s:"
ip -details link show can0 | grep -E "can0|bitrate"
echo
echo "Tip: 'candump can0' in another terminal shows every frame on the bus."
