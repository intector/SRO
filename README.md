# SRO-MK4 - Smart Reflow Oven Controller

> *"If you have knowledge, let others light their candles in it."* — Margaret Fuller

An ESP32-S3 based controller for converting a standard toaster oven into a precision soldering reflow oven. Features a responsive web interface, real-time temperature monitoring, and professional-grade reflow profile execution.

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-green.svg)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange.svg)

## Features

### Temperature Control
- **Dual-PID Architecture** - Separate heating and cooling controllers with mutual exclusion
- **Feedforward Compensation** - Accurate ramp tracking during soak phases
- **Bumpless Transfer** - Smooth transitions between heating and cooling modes
- **18-second Dead Time Handling** - Optimized for thermal lag in standard ovens
- **Boost Mode** - 100% heater output for rapid temperature ramps

### Profile Execution
- **4-Phase Reflow Profiles** - Preheat, Soak, Reflow, and Cooling
- **JSON Profile Storage** - Easy to create and modify profiles
- **Real-time Progress Tracking** - Visual feedback during soldering
- **Multiple Profile Support** - SAC305 lead-free, low-temp SAC, and custom profiles

### Web Interface
- **Real-time Temperature Chart** - Live visualization with Chart.js
- **WebSocket Communication** - Instant updates without polling
- **Profile Management** - Create, edit, and run profiles from the browser
- **PID Tuning Interface** - Adjust parameters without reflashing
- **Mobile Responsive** - Works on phones and tablets

### Hardware Drivers
- **MAX6675 Thermocouple** - K-type temperature sensing (0.25°C resolution)
- **Solid State Relay Control** - Software PWM for heater (500ms period)
- **Servo-Controlled Door** - Active cooling with smooth movement
- **APA102 Status LEDs** - Temperature-based color feedback
- **WS2812 Boot Status** - Visual boot sequence indicator

## Hardware Requirements

### Microcontroller
- ESP32-S3 DevKit or custom board
- 4MB+ Flash recommended
- WiFi connectivity

### Sensors & Actuators
| Component | Purpose | Interface |
|-----------|---------|-----------|
| MAX6675 + K-Type Thermocouple | Temperature sensing | SPI |
| Solid State Relay (SSR) | Heater control | GPIO |
| Solid State Relay (SSR) | Cooling fan control | GPIO |
| Servo Motor (SG90 or similar) | Door control | PWM |
| APA102 LEDs (optional) | Status indication | SPI |
| WS2812 LED (optional) | Boot status | RMT |

### Oven Requirements
- Standard toaster/convection oven (1000-1500W)
- Able to reach 250°C
- Door that can be servo-controlled or manually operated

## Pin Configuration

Default pin assignments (configurable in `main.h`):

```
Temperature Sensor (MAX6675):
  - MISO: GPIO 11
  - CLK:  GPIO 12
  - CS:   GPIO 13

SSR Control:
  - Heater: GPIO 4
  - Fan:    GPIO 5

Servo (Door):
  - PWM:    GPIO 6

APA102 LEDs:
  - MOSI:   GPIO 35
  - CLK:    GPIO 36

WS2812 LED:
  - Data:   GPIO 48
```

## Software Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Web Interface                        │
│              (HTML/CSS/JS + Chart.js)                   │
└─────────────────────┬───────────────────────────────────┘
                      │ WebSocket
┌─────────────────────▼───────────────────────────────────┐
│                  ESP32-S3 Firmware                      │
├──────────────┬──────────────┬──────────────┬────────────┤
│  WebSocket   │    HTTP      │     FTP      │   WiFi     │
│  Broadcast   │   Server     │   Server     │  Manager   │
├──────────────┴──────────────┴──────────────┴────────────┤
│                  Profile Manager                        │
│         (Phase state machine, timing control)           │
├─────────────────────────────────────────────────────────┤
│               Temperature Control                       │
│    (Dual-PID, Feedforward, Boost mode, Bumpless)        │
├──────────────┬──────────────┬──────────────┬────────────┤
│   MAX6675    │     SSR      │    Servo     │  APA102    │
│  (Temp)      │  (Heater)    │   (Door)     │  (LEDs)    │
└──────────────┴──────────────┴──────────────┴────────────┘
```

## Building & Flashing

### Prerequisites
- ESP-IDF v5.x installed and configured
- Python 3.8+
- USB driver for your ESP32-S3 board

### WiFi Configuration

Before building, create your WiFi credentials file:

```bash
cd ESP32/SRO-MK4/SRO/main
cp wifi_credentials.h.example wifi_credentials.h
```

Edit `wifi_credentials.h` with your network details:

```c
#define A2S_WIFI_DEFAULT_SSID        "your_wifi_ssid"
#define A2S_WIFI_DEFAULT_PASSWORD    "your_wifi_password"
#define A2S_WIFI_DEFAULT_HOSTNAME    "SRO-OVEN"
```

> ⚠️ **Important:** Never commit `wifi_credentials.h` to version control! It's already in `.gitignore`.

### Build Steps

```bash
# Clone the repository
git clone https://github.com/intector/SRO.git
cd SRO/ESP32/SRO-MK4/SRO

# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Configure the project (optional)
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### Web Interface Upload

The web interface files are served from the ESP32's FAT filesystem. Upload via FTP:

| Setting | Value |
|---------|-------|
| Host | ESP32's IP address |
| Port | 21 |
| Username | `sro` |
| Password | `upload` |

Using command line:
```bash
ftp 192.168.x.x
# Username: sro
# Password: upload

cd www
put index.html
# ... upload remaining files

Or use a GUI FTP client like FileZilla or WinSCP.

# Upload web files to /fatfs/www/
put index.html
put assets/*.*
put css/*.*
put js/*.*

# Upload profile files to /fatfs/profiles/
put profiles/*.*
```

## Configuration

### PID Parameters
Default values (tune for your specific oven):

| Parameter | Default | Description |
|-----------|---------|-------------|
| Kp | 3.5 | Proportional gain |
| Ki | 0.015 | Integral gain |
| Kd | 30.0 | Derivative gain |
| Feedforward Gain | 140.0 | Ramp compensation |
| Derivative Filter | 0.1 | Noise filtering |

### Profile Format

Profiles are stored as JSON in `/fatfs/profiles/`:

```json
{
  "id": 0,
  "name": "SAC305 Lead-Free",
  "description": "Standard lead-free solder profile",
  "solder_melting_point": 190,
  "absolute_peak_temp": 250,
  "preheat": {
    "max_temp": 150,
    "time_sec": 90
  },
  "soak": {
    "max_temp": 180,
    "time_sec": 120
  },
  "reflow": {
    "min_temp": 200,
    "max_temp": 230,
    "time_sec": 30
  },
  "cooling": {
    "max_rate": -3,
    "target_temp": 50
  }
}
```

## Usage

1. **Power on** - ESP32 boots, connects to WiFi, starts web server
2. **Open web interface** - Navigate to the ESP32's IP address
3. **Select profile** - Choose appropriate solder paste profile
4. **Place PCB** - Position board in oven with components
5. **Start profile** - Click RUN and monitor the temperature curve
6. **Wait for completion** - Door opens automatically for cooling
7. **Remove PCB** - Once below 50°C, remove the finished board

## Safety Considerations

⚠️ **This project involves high temperatures and mains electricity. Build and use at your own risk.**

- Always supervise the oven during operation
- Ensure proper ventilation for flux fumes
- Use appropriate SSRs rated for your heater current
- Implement hardware emergency stop if possible
- The firmware includes software safety limits but hardware failsafes are recommended
- Keep flammable materials away from the oven
- Allow adequate cooling before handling PCBs

## Troubleshooting

### ESP32-S3 Clone Boards - Boot Mode Issue

Some off-brand ESP32-S3 DevKit boards may enter BOOT mode on cold power-up (after extended power-off). This is caused by a weak internal pull-up on GPIO0.

**Fix:** Add an external 10kΩ resistor from GPIO0 to 3.3V.

## Project Structure

```
SRO/
├── ESP32/
│   └── SRO-MK4/
│       └── SRO/
│           ├── main/
│           │   ├── main.c                    # Application entry point
│           │   ├── main.h                    # Pin definitions, event groups
│           │   ├── SRO_TemperatureControl.c  # PID control, feedforward
│           │   ├── SRO_ProfileManager.c      # Profile execution state machine
│           │   ├── SRO_WebServer.c           # HTTP server
│           │   ├── SRO_WebSocketServer.c     # WebSocket handling
│           │   ├── SRO_WebSocketBroadcast.c  # Real-time data broadcast
│           │   ├── SRO_FtpServer.c           # FTP for file uploads
│           │   ├── SRO_SystemCoordinator.c   # System initialization
│           │   ├── a2s_pid.c                 # PID controller library
│           │   ├── a2s_max6675.c             # Temperature sensor driver
│           │   ├── a2s_ssr.c                 # SSR control driver
│           │   ├── a2s_servo.c               # Servo motor driver
│           │   ├── a2s_apa102.c              # LED strip driver
│           │   ├── a2s_wifi.c                # WiFi management
│           │   ├── wifi_credentials.h.example # WiFi config template
│           │   └── wifi_credentials.h        # Your WiFi config (not in repo)
│           ├── CMakeLists.txt
│           ├── partitions.csv
│           └── sdkconfig
├── www-WebStorm/
│   ├── index.html                # Main web page
│   ├── assets/
│   │   ├── css/                  # Stylesheets
│   │   ├── js/                   # JavaScript modules
│   │   ├── fonts/                # Web fonts
│   │   └── img/                  # Icons and images
│   ├── css/                      # Third-party CSS (Bootstrap)
│   └── js/                       # Third-party JS (jQuery, Chart.js)
├── profiles/                     # Reflow profile JSON files
├── pictures/                     # Project photos
├── LICENSE
└── README.md
```

## Technical Details

### Control Loop Timing
- PID update rate: 100ms
- Temperature sampling: 220ms (MAX6675 limit)
- WebSocket broadcast: 200ms
- Heater PWM period: 500ms

### Thermal Characteristics
- Dead time: ~18 seconds (heater response delay)
- Boost mode exit margin: 15°C (preheat), 5°C (reflow)
- Hysteresis band: ±1°C between heat/cool modes

### Memory Usage
- Heap: ~200KB free at runtime
- Flash: ~1.5MB firmware + FAT filesystem

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ESP-IDF team for the excellent framework
- Chart.js for the visualization library
- The reflow oven DIY community for inspiration
- Tim Wescott's "PID Without a PhD" for control theory guidance

## Version History

- **MK4** (Current) - Streamlined design, feedforward control, bumpless transfer
- **MK3** - Added web interface, dual-PID architecture
- **MK2** - Basic PID control, serial interface
- **MK1** - Proof of concept

---

*Built with ☕ and solder fumes*
