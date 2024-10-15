# IoT Smart Farming System

This repository contains the code for an IoT-based smart farming system. It integrates multiple sensors for environmental monitoring and uses MQTT for real-time communication, facilitated by the **CloudAMQ** provider.

## Overview

The system employs an **ESP32** microcontroller to gather data from the following sensors:
- **BH1750**: Measures light intensity.
- **DHT11**: Measures temperature and humidity.
- **Two NPK sensors**: Measure soil nutrient levels (Nitrogen, Phosphorus, and Potassium).

An **LCD display** is used to visualize data locally, where the right bottom corner shows the number of connected sensors.

## Code Structure

This project includes six files, with the final and most updated version being:
- **Mfarming_read_rev8**: The final version of the code. This is the file to flash onto your ESP32 device for deployment.

## Features

- **Light Intensity Monitoring**: Data from the BH1750 sensor is published via MQTT.
- **Temperature and Humidity Monitoring**: Data from the DHT11 sensor is published via MQTT.
- **Soil Nutrient Monitoring**: Data from the two NPK sensors is published via MQTT.
- **Relay Control**: MQTT subscriptions enable controlling external devices like irrigation systems and lighting through defined topics.
- **LCD Display**: The LCD shows sensor data, and the number of sensors is displayed in the right bottom corner.

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

## CloudAMQ Configuration

The system uses **CloudAMQ** as the MQTT broker provider. Ensure you configure your credentials (broker address, username, and password) in the code to enable MQTT communication.

## Hardware Requirements
- **Microcontroller**: ESP32
- **Sensors**:
  - BH1750 for light intensity
  - DHT11 for temperature and humidity
  - Two NPK sensors for soil nutrients
- **LCD Display**: For visualizing sensor data and indicating the number of sensors
- **Relays**: For controlling external devices

## Getting Started

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   ```

2. **Final Code**:
   The `Mfarming_read_rev8` file is the final version of the code. Use this file to flash onto your ESP32.

3. **Configure CloudAMQ**:
   Update the code with your CloudAMQ broker details (address, username, and password).

4. **Use the LCD Display**:
   The LCD will display real-time sensor data, with the sensor count visible in the bottom-right corner.

5. **Run the System**:
   Once deployed, the ESP32 will publish sensor data to the specified MQTT topics and receive commands from the subscribed relay topics.

## Dependencies

- **ESP32** board
- Libraries:
  - **Adafruit_BH1750** for the BH1750 sensor
  - **DHT sensor library** for the DHT11 sensor
  - **ModbusMaster** for NPK sensor communication
  - **PubSubClient** for MQTT communication
  - **LiquidCrystal_I2C** for the LCD display

## Future Enhancements

- Add additional MQTT topics for controlling more devices (e.g., `relay3`, `relay4`).
- Expansion to more sensors or automated farming systems.

## Contributing

Contributions are welcome! Please submit any issues or pull requests to help improve the system.

## License

This project is licensed under the MIT License.