#!/bin/bash

DEVICE1=/dev/ttyUSB0
DEVICE2=/dev/ttyUSB1

# 检查 root 权限
if [[ $EUID -ne 0 ]]; then
    echo "[WARN] This script must be run as root. Please use: sudo $0"
    exit 1
fi

# 检查 socat 是否存在
if ! command -v socat &> /dev/null; then
    echo "[WARN] socat not found. Please install socat."
    exit 1
fi

# 创建虚拟串口
(socat -d -d pty,raw,echo=0,link=${DEVICE1},mode=666 \
              pty,raw,echo=0,link=${DEVICE2},mode=666) &

sleep 0.5

echo "[INFO] Virtual serial ports created:"
ls -l ${DEVICE1} ${DEVICE2}

SOCAT_PID=$!

echo "[INFO] socat running in background with PID: $SOCAT_PID"

# 等待用户结束
read -p "[INFO] Press Enter to stop the virtual serial ports..."

kill -9 $SOCAT_PID

echo "[INFO] Virtual serial ports stopped."