# Embedded-Waveform-Synthesizer-PCBA
A full-stack synthesizer project featuring embedded waveform generation, custom PCB design, and detailed hardware, peripheral, and interrupt management using C.

## Overview
This project showcases a full-stack design for a synthesizer with embedded waveform generation and custom PCB assembly (PCBA). It combines embedded software development, analog circuitry design, and PCB layout to create a functional and enjoyable device. The synthesizer leverages various hardware components, peripherals, and real-time event handling to generate and manipulate analog waveforms.

## Purpose
The main objective of this project is to demonstrate proficiency in:
- Embedded software development
- Analog circuit design
- PCB design and assembly

## Features

### Hardware Configuration
The synthesizer's firmware initializes various hardware settings, including:
- Oscillator settings
- Pin directions
- Peripheral configurations

This setup is achieved through configuration bits and direct register manipulation.

### Peripheral Initialization
Several peripherals are initialized to facilitate the synthesizer's functionality:
- **Analog-to-Digital Converter (ADCC)**: Reads analog values for waveform generation.
- **Timer0 Module**: Manages time-based events for periodic waveform generation.
- **Serial Peripheral Interface (SPI)**: Enables communication with external devices, such as the Digital-to-Analog Converter (DAC).

### LCD Initialization
The project includes the initialization of an LCD display using a custom library (`ANY_LCD_H.h`). This library handles sending commands and data to the display, allowing for user interaction and feedback.

### DAC Communication
Functions are provided for communicating with a DAC via SPI. This communication is crucial for converting digital signals into analog waveforms.

### Interrupt Handling
Interrupts are used to handle Timer0 overflows, generating periodic waveforms and managing the main execution flow asynchronously. This ensures real-time responsiveness and efficient task management.

### Main Loop
The main loop of the firmware:
- Continuously checks for button presses
- Triggers actions such as reading analog values from the ADC
- Initializes Timer0 for waveform generation
- Resets the LCD display as needed

### Commentary and Documentation
The code is well-documented with comments explaining the purpose and functionality of each function and code block. This commentary aids in understanding, maintaining, and extending the codebase.

## Development Environment
The firmware is written in C, chosen for its ability to facilitate direct hardware communication and utilize CPU interrupts effectively. This low-level programming approach allows for fine-grained control over the synthesizer's operation.

## Conclusion
This project is a comprehensive example of firmware development for embedded systems. It demonstrates the integration of hardware initialization, peripheral communication, interrupt handling, and real-time event-driven programming. The synthesizer with embedded waveform generation and custom PCBA serves as a fun and practical device, illustrating the capabilities of full-stack embedded system design.

## Usage
To use the synthesizer, follow these steps:
1. Assemble the custom PCB according to the provided design files.
2. Flash the firmware onto the microcontroller.
3. Power up the device and interact with the buttons and LCD display to generate and manipulate waveforms.

## Future Work
Potential future enhancements for this project include:
- Expanding the range of waveforms generated
- Adding more user interface elements
- Integrating additional peripherals for extended functionality

By continuing to develop this project, it can serve as a robust platform for exploring advanced concepts in embedded systems and analog circuit design.
