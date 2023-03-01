# kidz

Find out more at [notes.skytect.one/robocup](https://notes.skytect.one/robocup)!

```bash
.
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
