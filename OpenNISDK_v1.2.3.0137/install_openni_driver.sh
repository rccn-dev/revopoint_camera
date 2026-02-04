#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define Colors for prettier output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting OpenNI2 Driver Installation for Revopoint/3DCamera...${NC}"

# 1. Check Root
if [ "$(id -u)" -ne 0 ]; then
    echo -e "${RED}Error: Please run this script with sudo.${NC}"
    exit 1
fi

# 2. Set up Paths
ORIG_PATH=$(pwd)
cd "$(dirname "$0")"
SCRIPT_PATH=$(pwd)
ARCH=$(uname -m)

# 3. Install System Dependencies
echo -e "${YELLOW}[1/6] Installing system dependencies...${NC}"
apt-get update -qq
apt-get install -y -qq libopenni2-dev openni2-utils wget curl

# 4. Handle the "Missing SSL 1.1" Issue (Ubuntu 22.04/24.04+)
echo -e "${YELLOW}[2/6] Checking for Legacy SSL 1.1...${NC}"
if ! ldconfig -p | grep -q libssl.so.1.1; then
    echo -e "      OpenSSL 1.1 not found (Required for this driver)."
    echo -e "      Downloading and installing automatically..."
    
    SSL_DEB="libssl1.1_1.1.1f-1ubuntu2.24_amd64.deb"
    SSL_URL="http://security.ubuntu.com/ubuntu/pool/main/o/openssl/${SSL_DEB}"
    
    wget -q -O /tmp/${SSL_DEB} ${SSL_URL}
    dpkg -i /tmp/${SSL_DEB}
    rm /tmp/${SSL_DEB}
    echo -e "${GREEN}      OpenSSL 1.1 installed.${NC}"
else
    echo -e "${GREEN}      OpenSSL 1.1 is already installed.${NC}"
fi

# 5. Determine Correct System Paths
# Finds where apt installed OpenNI2 (e.g., /usr/lib/x86_64-linux-gnu/OpenNI2/Drivers)
OPENNI2_DRIVER_DIR=$(dpkg -L libopenni2-0 | grep "/OpenNI2/Drivers$" | head -n 1)

if [ -z "$OPENNI2_DRIVER_DIR" ]; then
    echo -e "${RED}Error: Could not locate OpenNI2 Drivers directory. Is libopenni2-0 installed?${NC}"
    exit 1
fi

# Detect system library path (usually /usr/lib/x86_64-linux-gnu)
SYSTEM_LIB_DIR="/usr/lib/$(uname -m)-linux-gnu"
if [ ! -d "$SYSTEM_LIB_DIR" ]; then
    # Fallback for older systems
    SYSTEM_LIB_DIR="/usr/lib"
fi

echo -e "      Target Driver Dir: ${OPENNI2_DRIVER_DIR}"
echo -e "      Target System Lib: ${SYSTEM_LIB_DIR}"

# 6. Copy Files
echo -e "${YELLOW}[3/6] Copying Driver Files...${NC}"

# Copy the helper library to system libs
if [ -f "${SCRIPT_PATH}/lib/linux/lib3DCamera.so" ]; then
    cp "${SCRIPT_PATH}/lib/linux/lib3DCamera.so" "${SYSTEM_LIB_DIR}/"
    echo -e "      Copied lib3DCamera.so to ${SYSTEM_LIB_DIR}"
else
    echo -e "${RED}Error: Could not find lib/linux/lib3DCamera.so in source folder.${NC}"
    exit 1
fi

# Copy the actual driver plugin to OpenNI folder
if [ -f "${SCRIPT_PATH}/lib/linux/libcsdevice.so" ]; then
    cp "${SCRIPT_PATH}/lib/linux/libcsdevice.so" "${OPENNI2_DRIVER_DIR}/"
    echo -e "      Copied libcsdevice.so to ${OPENNI2_DRIVER_DIR}"
else
    echo -e "${RED}Error: Could not find lib/linux/libcsdevice.so in source folder.${NC}"
    exit 1
fi

# 7. Create Symlinks
echo -e "${YELLOW}[4/6] Creating Symlinks...${NC}"
ln -sf "${OPENNI2_DRIVER_DIR}/libcsdevice.so" "${OPENNI2_DRIVER_DIR}/libcsdevice.so.0"

# 8. Install Udev Rules
echo -e "${YELLOW}[5/6] Setting up USB Rules...${NC}"
if [ -f "${SCRIPT_PATH}/rules/cs_uvc.rules" ]; then
    cp "${SCRIPT_PATH}/rules/cs_uvc.rules" /etc/udev/rules.d/cs_uvc.rules
    udevadm control --reload-rules && udevadm trigger
else
    echo -e "${RED}Warning: rules/cs_uvc.rules not found. USB permissions might fail.${NC}"
fi

# 9. Refresh Linker Cache
echo -e "${YELLOW}[6/6] Refreshing Library Cache...${NC}"
ldconfig

echo -e "${GREEN}===============================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "You can now test the camera by running: ${YELLOW}sudo NiViewer2${NC}"
echo -e "${GREEN}===============================================${NC}"

cd "$ORIG_PATH"