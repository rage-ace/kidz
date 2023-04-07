#!/bin/bash
# Prepare to run
pkill -9 "python3"  # In case any existing instance of the server is running
cd $(dirname $0)/server

# Run
screen -dmS camera sudo python3 noserver.py
