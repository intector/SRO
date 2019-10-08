# SRO
## Soldering Reflow Oven

### The Web-Interface

This web page utilizes the Bootstrap 4 tools to be able to use devices with different screen sizes. JSON objects are used to format the data. The communication is managed by a secure websocket which is connected to the MQTT-Broker.

The file [mqtt_config.js](js/mqtt_config.js) contains the credentials for the MQTT-Broker authentication and must be adjusted according to your requirements.
