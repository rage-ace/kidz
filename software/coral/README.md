# Camera 📸

## Setup

```bash
 # Connect to a network to SSH over, and set hostname to "kidz.local"
sudo nmtui
sudo nano /etc/hosts  # Also change your hostname here to "kidz"

# Setup autofocus
snapshot  # Send 'r' to focus once
# The run scripts disable autofocus

# Install python dependencies
sudo python3 -m pip install -r requirements.txt  # We will be running the script with root

# Set up service
sudo apt install -y screen
sudo cp camera.service /etc/systemd/system/
sudo systemctl enable camera  # TODO: THIS DOES NOT WORK (https://stackoverflow.com/questions/75961521/pyserial-write-raises-errno-5-input-output-error-a-while-after-startup-when)
```

## Execution

```bash
# Run the version with a webserver debugger
# Access it at http://kidz.local:8080
sudo sh run_server.sh

# Run the headless version
sudo sh run_noserver.sh
```

## Development

```bash
# Install dependenceis
sudo apt install -y python3-dev
python3 -m pip install --user black pipreqs
echo 'export PATH="/home/mendel/.local/bin:$PATH"' >> ~/.bashrc

pipreqs . --force  # Capture python dependencies to requirements.txt
black .  # Reformat all code
```
