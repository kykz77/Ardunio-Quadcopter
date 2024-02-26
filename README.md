# Arduino-Quadcopter
 Hello! Welcome to my Arduino Quadcopter project!
 
 I am using an Arduino Uno as a flight controller to fly a quadcopter. In this project I make use of pin change interrupts, an IMU (gyroscope and accelerometer), ESCs, a Kalman filter, and a PID controller to fly the quadcopter. 

 Although I am currently experiencing some difficulty in tuning my PID controller to get the quadcopter to perfectly stabilise, the quadcopter is able to recieve and respond to RC signals, and make use of the gyroscope and accelerometer data to determine which directions it must move in to stabilise itself during flight. 
