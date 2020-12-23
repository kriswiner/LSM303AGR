# LSM303AGR
ST's combo accel/magnetometer (e-compass), sketch for Arduino

Basically two different sensors packaged together, I have treated the accel and mag as separate sensors. There are separate methods for each, one for the LSM303AGR accelerometer and one for the LIS2MDL magnetometer, sensible and convenient. The sketch configures the sensors, selects sample rates and full scale ranges, performs the self test, calibrates the sensors, and sets up interrupts for data ready and activity (no motion) detection, etc. Intended to run on the CMWX1ZZABZ (STM32L082 + SX1276) LoRaWAN-enabled MCU but can be easily modified for use with any I2C-enabled MCU.
