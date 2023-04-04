#!/bin/bash
sudo chmod a+rw /dev/ttyS0  # Required for serial write permissions
pkill -9 "python3 main.py"  # In case any existing instance of the server is running

cd $(dirname $0)/server
python3 server.py
