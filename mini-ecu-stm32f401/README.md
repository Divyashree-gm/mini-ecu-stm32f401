# Mini-ECU: Bare-metal Drive Control System (STM32F401)

A small embedded drive controller written in C with **direct register access only**: no HAL, no RTOS, no Arduino. It runs on an STM32F401 (default pin map for the NUCLEO-F401RE) from the 16 MHz internal oscillator (HSI).

A potentiometer sets the motor speed, a touch input enables the system, an emergency-stop button works through an interrupt, and a UART console gives commands and status.

## Features

- **Four-state machine:** IDLE, READY, RUNNING, EMERGENCY. Each state has its own LED, and LEDs are set in one place only, so they always match the state.
- **Interrupt-driven emergency stop:** the E-STOP interrupt (EXTI13, falling edge) turns PWM off first, then changes state.
- **Potentiometer to PWM:** ADC1 channel 0, 8-sample average, dead zones at both ends, mapped to 0 to 100 % duty on a 1 kHz PWM (TIM3_CH1).
- **UART console** at 115200 8N1 with a receive interrupt and ring buffer. Commands are case-insensitive and are accepted after Enter, or automatically after 1.5 s without input.
- **Safe state changes:** transitions run with interrupts briefly disabled, so they cannot race the E-STOP handler. Anything other than RUNNING forces the motor off.
- **Touch START** with rising-edge detection and 200 ms debounce.
- **1 ms system tick** (SysTick) for timestamps in the log and for uptime.

## Pin map

| Function | Pin | Notes |
|---|---|---|
| LED orange / green / blue / red | PB12 / PB13 / PB14 / PB15 | Orange = IDLE, green = READY, blue = RUNNING, red = EMERGENCY |
| Touch START | PA1 | Active high, pull-down |
| E-STOP | PC13 | EXTI13, falling edge, active low |
| Potentiometer | PA0 | ADC1_IN0 |
| PWM output | PA6 | TIM3_CH1, AF2, 1 kHz |
| Motor direction | PB0, PB1 | Direction inputs of the motor driver |
| UART | PA2 (TX), PA3 (RX) | USART2, 115200 8N1 |

## States

| State | LED | Meaning |
|---|---|---|
| IDLE | Orange | System not enabled. Touch START to enable. |
| READY | Green | Enabled, motor off. |
| RUNNING | Blue | Motor on, speed follows the potentiometer. |
| EMERGENCY | Red | E-STOP pressed. Motor off until RESET. |

## UART commands

| Command | Action |
|---|---|
| `F` | Run forward at the potentiometer speed |
| `R` | Run in reverse at the potentiometer speed |
| `S` | Stop and return to READY |
| `STATUS` | Print state, motor, direction, PWM, ADC value, emergency flag and uptime |
| `RESET` | Clear an emergency (the E-STOP button must be released first) |
| `HELP` | List the commands |

## Building

`src/main.c` holds the application only. To build it you also need the usual STM32F401 startup file (vector table) and linker script from your toolchain, for example from STM32CubeIDE or an `arm-none-eabi-gcc` project. The vector table must point to `SysTick_Handler`, `USART2_IRQHandler` and `EXTI15_10_IRQHandler`, which are defined in `main.c`.

## Hardware used

- NUCLEO-F401RE (or any STM32F401 board; change the pin map in the settings block at the top of `main.c`)
- Potentiometer on PA0
- Touch sensor or button on PA1
- Emergency-stop button on PC13 (the Nucleo user button is on this pin)
- Motor driver with direction inputs on PB0 and PB1 and PWM on PA6
- Four LEDs on PB12 to PB15
