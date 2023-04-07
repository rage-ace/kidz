#!/bin/bash
# Run this bit as root
sudo chmod a+rw /dev/ttyS0  # Required for serial write permissions

# Prepare to run
pkill -9 "python3"  # In case any existing instance of the server is running
cd $(dirname $0)/server

# Run
python3 main.py
