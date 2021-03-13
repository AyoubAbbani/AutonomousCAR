# AutonomousCAR


## Autonomous car based on AI

### Description

The project is built based on both Arduino and raspberryPi3
The Pi is hooked up to the Raspberry Pi Camera Module and an ultrasonic sensor.
one program writing in C++ on the Pi is used to serve the information it gathers from those sensors and then to transform them as commands to the Arduino Uno and send them with parallel communication in the other hand another program in Arduino is able to distribute the information received from Pi to all car actuators and using streaming video with remote desktop connection to run the Pi's program.

Note that u have to train your own datasete based on Cascade-Trainer

#### Features ( 4 ADAS Combined)

#####     - Lane Detection.
#####     - Traffic & Light Detection( stop sign & red light)
#####     - Front obstacle detection
#####     - Pedestrian Detection

##### Tech & used tools : Arduino Uno | RaspberryPi3 | RaspiCam | HC-SR04 | C/C++ | Machine learning  | OpenCv| raspbian os

##### Steps

 RasbianOS => Optmize the os => install requirements (Opencv,gcc,wiringP) => Values Calibration => train ur dataset => test
 
 
 ##### HW Architecture 
 
 ![shéma](https://user-images.githubusercontent.com/63366676/111030422-73a61f80-8402-11eb-83e8-99d93537682c.PNG)


##### SW Architecture 

![image](https://user-images.githubusercontent.com/63366676/111030434-86b8ef80-8402-11eb-9b2d-ffe78269cf56.png)

 
 
 
