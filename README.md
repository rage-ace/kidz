# kidz

Find out more at [notes.skytect.one/robocup](https://notes.skytect.one/robocup)!

```bash
.
├── designs/               # All CAD designs are located here
│   └── pcb-schematics/        # PCB schematics
│       ├── layer1/                # Layer 1 PCB
│       ├── layer2/                # Layer 2 PCB
│       ├── layer3/                # Layer 3 PCB
│       └── layer4/                # Layer 4 PCB
├── software/              # All code is located here
│   ├── coral/             # Code for the Coral Dev Board Mini
│   │   └── ...
│   └── microcontrollers/  # Code for the Teensy 4.0 and STM32F103CBT6s
│       ├── include/           # Shared header files
│       ├── lib/               # Shared libraries
│       ├── src/
│       │   ├── stm32_mux/         # Code for the Layer 1 STM32
│       │   │   ├── include/           # Header files
│       │   │   ├── lib/               # Libraries
│       │   │   ├── main.cpp           # Source code
│       │   │   └── ...
│       │   ├── teensy/            # Code for the Layer 3 Teensy
│       │   │   ├── include/           # Header files
│       │   │   ├── lib/               # Libraries
│       │   │   ├── main.cpp           # Source code
│       │   │   └── ...
│       │   └── ...            # Shared source code
│       ├── platform.ini       # PlatformIO configuration file
│       └── ...
├── .gitignore
├── LICENSE
└── README.md
```

## Coral

### Execution

```bash
cd software/coral
python3 run_server.py
```

### Saving Dependencies

```bash
pip3 install pipreqs
cd software/coral
pipreqs .
```

## Microcontrollers

### Execution

```bash
cd software/microcontrollers
# Upload and run to Teensy
pio run -e teensy -t upload -t monitor
# Upload to Layer 1 STM32
pio run -e stm32_mux -t upload
```
