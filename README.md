```# Obstacle avoidance robot car
## Objectives

The main objective of the project is to build a robot which will be capable to move freely avoiding obstacle in its
path without colliding with objects coming in its path,obstacle can be avoided using `ultrasonic sensor`.

## Hardware equipments:

Hardware equipments to build the robot:

 - Arduino MEGA Board
 - Ultrasonic Sensor – HC – SR04  
 - L298N Motor Driver Module 
 - Servo Motor ( SG90)  
 - 4 x 5V Geared Motors 
 - Robot Chassis  
 - Power Supply 
 - Battery Connector
 - Battery Holder
 - Connecting Wires

 ### Arduino 

 Arduino is used as the brain of the Robot, it is responsible for the decisions and the actions to be performed by the robot.
 The Arduino's board used in the project is `Arduino Mega 2560`.
 
 [Arduino_mega_2560](https://telmal.com/pl/p/Klon-Arduino-MEGA2560-R3-Atmel-ATMega2560-AVR-USB-/1023)

### Ultra sonic Sensor

Robot can use many type of sensors in the same time, this can be because different sensors detect different
materials in the non-crossing zones etc.

In the robot, I have used 1 type of ultra sonic sensor `HC – SR04`:
 - HC – SR04: It is an Ultrasonic Range Finder Sensor used to detect obstacle. It is a non-contact based distance 
 measurement system and can measure distance from 2cm to 4m.

[Ultrasonic_sensor_HC-SR04](https://geeksvalley.com/en/product/ultrasonic-sensor/)

### Servo Motor 

A servomotor is a **rotary actuator** that allows for precise control the angular position, in this project it is
used to rotate the `Ultrasonic Sensor` to change the direction of the robot.
The type of servo motor used in the project is `SG90` is a simple Servo Motor which can rotate 90 degrees in each 
direction (approximately 180 degrees in total). 

[Servo_motor_SG90](https://circuit.rocks/tower-pro-9g-sg-90-mini-servo)

####  L298N Motor Driver Module 

The L298N Motor Driver Module is responsible for providing the necessary drive current to the motors of the robotic car.
In this project it is used to control the speed and direction of the motors at the same time, to move the robot forward or backward.

[L298N_motor_driver_module](https://components101.com/modules/l293n-motor-driver-module)


## How to start 

Assemble the Robot, make the necessary connections and upload the code to **Arduino Mega 2560**.

 - [Four_wheels_robot_kit](https://pl.aliexpress.com/item/32719641064.html?gatewayAdapt=glo2pol)
 - [Obstacle_avoidance_wiring_diagram](resources/obstacle_avoidance_wiring_diagram.png)
 - [Download_arduino_IDE](https://www.arduino.cc/en/software)

## Working

When the robot is switched on,all the motors of the robot will run normally and the robot starts moving forward. 
During this time, the ultrasonic sensor continuously keep calculating the distance between the robot and the obstacle.

This information is processed by the Arduino. If the distance between the robot and the obstacle is less than 20 cm,
the Robot stops  and get back and waits 500 ms and stop again.

If the distance between the Robot and obstacle is greater than 20 cm the Robot keep going forward without hitting any obstacle, 
this process continues forever and the robot keeps moving without danger.

[Obstacle_avoidance_demostration_video](https://www.youtube.com/watch?v=ScbohdCsyO8)
