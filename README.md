# SRO
## Soldering Reflow Oven

This is the public repository for the SRO project. Please consider it as **_Work in progress_** and check back frequently for changes, you're also welcome to contribute to the project. I made this Soldering Reflow Oven as a tool which I needed for a different project on which I'm currently working. The parts are so small that it became impossible to assemble without using this method of soldering. I’m aware that there many devices out there which are probably cheaper as this one, but I just wanted always to make one of those things. I read many instructions and watched even more tutorials about **_DIY Soldering Reflow Ovens_** which is probably the reason why you’ll find the one or other thing familiar. This is not only true for the hardware, parts of the software are also rooted in other projects. As I already mentioned the SRO project does exist because I needed a tool and this is the reason I used mainly parts which I had laying around.

### Here’re some technical data of the SRO:

* **_SRO_**
  * up to 10 different soldering profiles
  * dynamic real time process data display
  * PID controller for precise temperature control
  * 4 quartz heating elements
  * infrared shield to protect PCB and components
  * convection fan to prevent "hot spots"
* **_Controller - ESP32-NodeMCU_**
  * Xtensa® dual-core 32-bit LX6 microprocessors
  * single 2.4 GHz Wi-Fi-and-Bluetooth combo chip
* **_Web-Interface_**
  * responsive Bootstrap 4 toolkit
  * JavaScript for execution on client site
  * JSON data objects
  * MQTT client for communication
* **_Communication and data exchange via MQTT-Broker_**
  * Mosquitto MQTT-Broker
  * Synology DS713+ as host machine
  * SSL encryption with 2048-bit key


### project directories:

* **_www_**
  * contains the web interface with all libraries, pictures and style sheets
* **_pictures_**
  * contains pictures of the SRO and parts which are used
* **_ESP32_**
  * contains the files for the ESP32 firmaware with it's source code
 
 
 
