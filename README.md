# AHDS-STM32
altitude hold drone simple but on an stm32. Inspired by joop brokking's videos
I took the method for creating pwms and reading interrupts from joop brokking's videos.
The rest of the code is my own.
One more thing: this code has an update rate of 400Hz(just like the arduino version), Uses a 1Khz clock speed for the I2c communications
The cpu clock speed is 128MHz(overclocked). Use my forked version of stm32 library and update your stm32 package in arduino 
More features will be added later(adding bmp280 so that i can fuse it's data with the ultrasonic sensor's data and get better altitude estimates, position control using optical flow(will use SPI at 4MHz, use my forked version of the stm32 library), GPS(will use UART 2) and MARG(mag,accel, rate, gravity, basically gon add magnetometer for getting the absolute orientation). Once the position control stuff is done I will add trajectory planning(as i have in the "self driving car" project) and add obstacle avoidance (which i have to add to the self driving car first :P). 
Eventually i plan to make this drone capable of doing swarm stuff but not like formations, more like mission based swarms in which the drones have a particular objective(maybe use single shot detectors for detecting stuff). Haven't really thought about it yet. more concerned with making a reliable flight controller first :P.  
