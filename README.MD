Here’s a README documentation tailored to your GitLab project:

---

# IoT Smart Farming System

This repository contains the code for an IoT-based smart farming system. The project uses sensors to monitor environmental conditions and communicates via MQTT for real-time data transmission and control.

## Overview

The system utilizes the ESP32 microcontroller to collect data from the following sensors:
- **BH1750**: Measures light intensity.
- **DHT11**: Measures temperature and humidity.
- **Two NPK sensors**: Measure soil nutrient levels (Nitrogen, Phosphorus, and Potassium).

The system supports MQTT communication with three data publishing topics and six control subscribing topics.

## Code Structure

The project includes six files, with the final and most updated version being:
- **Mfarming_read_rev8**: The final version of the code. Use this file for deployment to the ESP32.

## Features

- **Light Intensity Monitoring**: Publishes data from the BH1750 sensor.
- **Temperature and Humidity Monitoring**: Publishes data from the DHT11 sensor.
- **Soil Nutrient Monitoring**: Publishes data from two NPK sensors.
- **Relay Control**: Supports controlling devices like irrigation systems and lights through MQTT topics.

## MQTT Topics

### Publish Topics (Sensor Data)
- **farm/bht**: Publishes light intensity data from the BH1750 sensor.
- **farm/npk1**: Publishes soil nutrient data from NPK sensor 1.
- **farm/npk2**: Publishes soil nutrient data from NPK sensor 2.

### Subscribe Topics (Relay Control)
- **relay1**: Controls the first relay.
- **relay2**: Controls the second relay.
- **relay3**: Controls the second relay.
- **relay4**: Controls the second relay.
- **relay5**: Controls the second relay.
- **relay6**: Controls the second relay.

## Hardware Requirements
- **Microcontroller**: ESP32
- **Sensors**:
  - BH1750 for light
  - DHT11 for temperature and humidity
  - Two NPK sensors
- **Relays**: For controlling external devices
- **LCD**: For display the result

## Getting Started

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   ```

2. **Use the Final Code**:
   The `Mfarming_read_rev8` file is the final version of the code. Flash this file to your ESP32 device.

3. **Set Up MQTT**:
   Configure your ESP32 to connect to your MQTT broker by entering the broker’s details (IP address, port, username, and password) in the code.

4. **Run the System**:
   Once the ESP32 is running, sensor data will be automatically published to the corresponding MQTT topics, and you can control the relays via the subscribe topics.

## Dependencies

- **ESP32** board and development environment
- Required Libraries:
  - **Adafruit_BH1750** for light sensor
  - **DHT sensor library** for temperature and humidity
  - **ModbusMaster** for NPK sensor communication
  - **PubSubClient** for MQTT communication

## Future Enhancements

- Additional MQTT topics for more device control (e.g., `relay3`, `relay4`).
- Expansion to other sensors or automation systems.

## License

This project is licensed under the MIT License.

---

This documentation gives a clear structure and guides users through the setup and use of the project, with a focus on the final `Mfarming_read_rev8` file. You can adjust the repository URL and further customize the instructions as needed.Here’s a README documentation tailored to your GitLab project:

---

# IoT Smart Farming System

This repository contains the code for an IoT-based smart farming system. The project uses sensors to monitor environmental conditions and communicates via MQTT for real-time data transmission and control.

## Overview

The system utilizes the ESP32 microcontroller to collect data from the following sensors:
- **BH1750**: Measures light intensity.
- **DHT11**: Measures temperature and humidity.
- **Two NPK sensors**: Measure soil nutrient levels (Nitrogen, Phosphorus, and Potassium).

The system supports MQTT communication with three data publishing topics and six control subscribing topics.

## Code Structure

The project includes six files, with the final and most updated version being:
- **Mfarming_read_rev8**: The final version of the code. Use this file for deployment to the ESP32.

## Features

- **Light Intensity Monitoring**: Publishes data from the BH1750 sensor.
- **Temperature and Humidity Monitoring**: Publishes data from the DHT11 sensor.
- **Soil Nutrient Monitoring**: Publishes data from two NPK sensors.
- **Relay Control**: Supports controlling devices like irrigation systems and lights through MQTT topics.

## MQTT Topics

### Publish Topics (Sensor Data)
- **farm/bht**: Publishes light intensity data from the BH1750 sensor.
- **farm/npk1**: Publishes soil nutrient data from NPK sensor 1.
- **farm/npk2**: Publishes soil nutrient data from NPK sensor 2.

### Subscribe Topics (Relay Control)
- **relay1**: Controls the first relay.
- **relay2**: Controls the second relay.
- **Future Topics**: More control topics will be added to support additional devices.

## Hardware Requirements
- **Microcontroller**: ESP32
- **Sensors**:
  - BH1750 for light
  - DHT11 for temperature and humidity
  - Two NPK sensors
- **Relays**: For controlling external devices

## Getting Started

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   ```

2. **Use the Final Code**:
   The `Mfarming_read_rev8` file is the final version of the code. Flash this file to your ESP32 device.

3. **Set Up MQTT**:
   Configure your ESP32 to connect to your MQTT broker by entering the broker’s details (IP address, port, username, and password) in the code.

4. **Run the System**:
   Once the ESP32 is running, sensor data will be automatically published to the corresponding MQTT topics, and you can control the relays via the subscribe topics.

## Dependencies

- **ESP32** board and development environment
- Required Libraries:
  - **Adafruit_BH1750** for light sensor
  - **DHT sensor library** for temperature and humidity
  - **ModbusMaster** for NPK sensor communication
  - **PubSubClient** for MQTT communication

## Future Enhancements

- Additional MQTT topics for more device control (e.g., `relay3`, `relay4`).
- Expansion to other sensors or automation systems.

## Contributing

If you'd like to contribute to this project, feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License.
