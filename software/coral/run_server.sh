#!/bin/bash
# Prepare to run
pkill -9 "python3"  # In case any existing instance of the server is running
echo 0 > /sys/module/ov5645_camera_mipi_v2/parameters/ov5645_af # Disable autofocus
cd $(dirname $0)/server

# Run
sudo python3 main.py
