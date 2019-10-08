# SRO
## Soldering Reflow Oven

### Here’re the important parts I used:

#### Toaster Oven - BLACK+DECKER TO3250XSB
![Toaster Oven](/pictures/parts/oven.jpg)
BLACK+DECKER 8-Slice Extra-Wide Stainless Steel/Black Convection Countertop Toaster Oven, Stainless Steel, TO3250XSB. I bought it at [Walmart](https://www.walmart.com/ip/BLACK-DECKER-8-Slice-Extra-Wide-Stainless-Steel-Black-Convection-Countertop-Toaster-Oven-Stainless-Steel-TO3250XSB/34516916) for $60. It has 4 quartz infrared heating elements and a convection fan which is exactly what I was looking for. Other toaster ovens will probably work as Soldering Reflow Oven, but this one was just the one I choose.

---

#### Controller - ESP32-NodeMCU
![Controller Module](/pictures/parts/NodeMCU.jpg)
I got those from China quite some time ago and can't recall from where exactly, but any ESP32 module should work for that purpose. The difference to the ESP32-DevKitC from ESPRESSIF is the additional blue LED which is connected to GPIO2. 

---

#### Temperature sensor -  MAX6675 Type K Thermocouple
![Temperature Module](/pictures/parts/MAX6675.jpg)
Those modules coming normally with a sensor which is encased in a screw looking housing. I decided not to use the original sensors because of their high inertia on temperature changes. To get the best possible sensor dynamic I used sensors without any housing.
![Temperature Module](/pictures/parts/thermocouple_sensor.jpg)

---

#### Front Status LED - APA102C
![LED](/pictures/parts/APA102C.jpg)
Since the machine itself don't have any display or buttons I decidedt to utilize some APA102C LED's which i had laying around from another project. Those LED communicating with a SPI inteface which makes them perfect for this application. 

---

#### Servo Motor - EYESKY 15KG
![Servo Motor](/pictures/parts/servo.jpg)
This motor is used to open the door after the soldering sequence to shorten the cool down time. It also opens the door during the cycle in case the temperature rises over the maximum tolerance of the actual set value to prevent damage on the PCB or parts. Any servo motor with sufficient torque will work for that purpose.


