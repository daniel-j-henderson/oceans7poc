# ocean's 7 PoC
STM32 Code for Proof of Concept

This code performs UART communications between the host PC and the STM32F767 chip (UartHandle, printf reroutes here) as well as to an Arduino via UartAux. 
A PWM is also enabled to use to control the ESC (electronic speed controller) for proof of concept tests. The PWM is configured such that a 50% duty cycle yields a ~1.5ms pulse (stationary), and changing the duty cycle controls the motor speed. 
The STM Hardware Abstraction Layer (HAL) libraries are used (and edited for custom interrupt handling). 
