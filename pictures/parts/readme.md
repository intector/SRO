# SRO
## Soldering Reflow Oven

### Here’re the important parts I used:

#### Controller - ESP32-NodeMCU
![Temperature Module](/pictures/parts/NodeMCU.jpg)
I got those from China quite some time ago and can't recall from where exactly, but any ESP32 module should work for that purpose. The difference to the ESP32-DevKitC from ESPRESSIF is the additional blue LED which is connected to GPIO2. 

---

#### Temperature sensor -  MAX6675 Type K Thermocouple
![Temperature Module](/pictures/parts/MAX6675.jpg)
Those modules coming normally with a sensor which is encased in a screw looking housing. I decided not to use the original sensors because of their high inertia on temperature changes. To get the best possible sensor dynamic I used sensors without any housing.
![Temperature Module](/pictures/parts/thermocouple_sensor.jpg)

---

#### Front Status LED - APA102C
![Temperature Module](/pictures/parts/APA102C.jpg)
Since the machine itself don't have any display or buttons I decidedt to utilize some APA102C LED's which i had laying around from another project. Those LED communicating with a SPI inteface which makes them perfect for this application. 

---

