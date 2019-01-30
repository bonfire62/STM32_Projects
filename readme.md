

# STM32_Projects
These C files are practice files for various embedded systems projects. The will not workstandalone, as they need the accompanying header files, and libraries. They are not meant for general use outside of my personal exercise and practice.
Board: STM32F072B-Disco
Microprocessor: STM32F072RBTx

The goal of these projects is to provide bare-metal programming practice.

LED_w_debounce_button.c - LED's toggling through debounced user-button control.

NVIC_External Button Interrupt.c - Use of external button interrupt to flash LED's through SYSCONF 

I2C_Gyroscope_communication_LEDS - Change the LED's based on the I2C communication from Gyroscope

Motor Controller.c	- controls the torque of a motor using an encoder and PID response
Motor Controller.h

USART LED Controller.c - Controls LED's from USART input text buffer
