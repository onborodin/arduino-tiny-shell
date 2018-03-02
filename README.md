

# Arduino tiny shell

Here I present sample arduino micro-application with
- ring buffer for input and output with hook to interrupts (wide shared ring buffer code with my adaptation and inmprovement)
- extensible tiny sheel for command execution
- some data manipulation function
- PWM timers and interrupts sample

Shell really is tiny and used only ~850 bytes of microcontroller code.

Well, if in truth, I wrote this code for the toys of my youngest son =)

We cann add own function call. The agreement about the called functions is very simple:
- function must accept string arguments `uint8_t*`
- and return an integer, now is `int16_t`


The syntax of the shell is also simple:

`command [arg [arg]];`

where `arg` can be string or signed/unsigned integer. There is nothing 
difficult to add your types. To do this, you need to write the translation
 functions that are called inside the command handler function.

The argument delimiter is a space, the command terminator is a semicolon `;`.

## This repository also contains

- adapted I2C driver code from Peter Fleury <pfleury@gmx.ch>
- code for lot of 1602 LCD over I2C with own frame buffer and terminal-like scrolling
- MD5 code for authentification procedures from https://git.cryptolib.org/arm-crypto-lib.git
- initial driver for RTC DS1307 over I2C
- sample of I2C slave register machine
- driver for MPU6050 IMU with Sebastian Madgwick and Robert Mahony algorithms http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- some other (SPI, ADC basic sample, etc)

## Arduino screen

![](http://wiki.unix7.org/_media/c/screenshot-2018-02-12-09-58-27.png)












