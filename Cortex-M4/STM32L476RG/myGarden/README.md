# myGarden
## About
**myGarden** is a cross-platform project that aims to implement electronic control for a smart gardening solution, measuring the environmental conditions that impact a plant's health (ambient temperature, relative air/soil humidity or luminosity), and controlling a series of actuators (irrigation pump, humidifier) in an attempt to regulate them.

## Platform
The inital prototype will be based on the [STMicroelectronics NUCLEO-L476RG](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html) evaluation board. This board is based on a [STM32L476RG](https://www.st.com/en/microcontrollers-microprocessors/stm32l476rg.html) MCU, a low-power ARM Cortex-M4 with single precision FPU and operating at a frequency of up to 80 MHz.

## Development tools
This project is being developed using a [VSCode](https://code.visualstudio.com/) based IDE, with [GNU Make](https://www.gnu.org/software/make/) and the [GNU ARM Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) for compilation. This combination ensures a high level of portability with low effort.