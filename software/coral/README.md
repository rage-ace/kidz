# Camera 📸

## Setup

```bash
sudo nmtui  # Connect to a network to SSH over, and set hostname to "kidz.local"
sudo nano /etc/hosts  # Also change your hostname here to "kidz.local"
pip3 install --user -r requirements.txt  # Install python dependencies

# Add to crontab
sudo crontab -e
# Add the following line:
# "@reboot ~/kidz/software/coral/enable_serial.sh"
crontab -e
# Add the following line:
# "@reboot /bin/sleep 1 && screen -dmS cam ~/kidz/software/coral/run_noserver.sh"
```

## Execution

```bash
# Run the version with a webserver debugger
# Access it at http://kidz.local:8080
./run_server.sh

# Run the headless version
./run_noserver.sh
```



## Development

```bash
pipreqs . --force  # Capture python dependencies to requirements.txt
black .  # Reformat all code
```
