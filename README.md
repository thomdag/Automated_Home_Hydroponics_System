# Automated Home Hydroponics System

An all-in-one automated solution designed to manage and optimize a home hydroponics system. The system seamlessly integrates sensor feedback, light controls, and visual plant health monitoring to maintain optimal conditions for plant growth.

## Features

- **Sensor Monitoring**: Continuously monitor the water temperature, TDS, and pH levels in the hydroponics solution.
  
- **Light Control**: Adjust lighting based on user-defined criteria or real-time sensor feedback.
  
- **Medium Quality Modification**: Ensure the hydroponic medium is always at its best by making real-time adjustments based on sensor feedback.
  
- **Real-Time Clock (RTC) Integration**: Utilizes RTC to track and display precise timestamps for events and logs.
  
- **Hydroponics Technique Compatibility**: Designed to work efficiently with both Deep Water Culture (DWC) and Nutrient Film Technique (NFT) setups.
  
- **Visual Plant Health Monitoring**: 
    - **Raspberry Pi Integration**: Incorporates a Raspberry Pi Zero 2 to act as the neural centre for visual monitoring.
    - **Camera Analysis**: Uses a camera module to capture periodic images of the plants.
    - **TensorFlow-powered Insights**: With the help of TensorFlow, the system examines the captured images to identify early signs of plant stress, potential diseases, or nutrient deficiencies.

#### Software:

- **Arduino IDE**: For programming and interfacing with the ESP32.
- **TensorFlow (on Raspberry Pi)**: Used for machine learning and image analysis. 

#### Hardware:

- **ESP32 Board**: Acts as the core microcontroller for the hydroponics setup.
- **Sensors**: Required sensors for monitoring water temperature, TDS, and pH.
- **RTC Module**: For real-time event logging.
- **Light Control Hardware**: Relay or equivalent modules to manage light conditions.
- **Raspberry Pi Zero 2**: Serves as the hub for visual plant health monitoring.
- **Camera Module**: Compatible with Raspberry Pi Zero 2 for capturing high-quality images of plants.