#!/bin/bash
# Aquila/setup.sh

cd /src/PX4-Autopilot
make distclean
git submodule update --init --recursive
./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# --- Configure DDS Client to find the ROS container ---
echo "uxrce_dds_client stop" > /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rc.dds_custom
# Connect to localhost on UDP port 8888
echo "uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888" >> /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rc.dds_custom

echo "Setup complete. DDS Client configured for Docker."