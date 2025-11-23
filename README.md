# STM32F446RE Peripheral Drivers

This repository contains a collection of **bare-metal peripheral drivers** implemented from scratch  
for the **STM32F446RE** microcontroller, without using HAL or LL libraries.

All drivers are written manually (register–level) with clean modular structure,  
and include multiple example applications demonstrating how each peripheral works.

---

## Features

### Custom Peripheral Drivers (Register-Level)
Implemented fully from scratch in C:

- **GPIO Driver**  
  Pin modes, speed configuration, pull-up/pull-down, interrupts (EXTI), output control.

- **SPI Driver**  
  Master mode, full-duplex communication, TX-only mode, interrupt-driven receiving.

- **I2C Driver**  
  Master transmit/receive, ACK handling, repeated start, blocking & interrupt variants.

- **USART Driver**  
  Basic TX/RX, simple send routines, polling-based communication.
