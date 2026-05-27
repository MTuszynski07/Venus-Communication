# I2C Multiplexer - TCA9548A guide

<img width="405" height="179" alt="image" src="https://github.com/user-attachments/assets/f517f64e-d5ae-4783-a1e4-0f0dc40176b0" />

We are using two VL53L0X ToF (distance) sensors and one MLX90614-DCC IR temperature sensor.  
All of these sensors use I2C, so we must use the I2C multiplexer.

# Wiring

<img width="556" height="694" alt="image" src="https://github.com/user-attachments/assets/df64c55b-3cca-417c-a456-795b7ab06608" />

I used channels 0, 2, and 4 for the sensors to make the wiring more organized and less cluttered.
