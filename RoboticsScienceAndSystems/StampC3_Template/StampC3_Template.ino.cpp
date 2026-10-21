# 1 "C:\\Users\\c85-lau\\AppData\\Local\\Temp\\tmpzgwc1i23"
#include <Arduino.h>
# 1 "C:/Users/c85-lau/OneDrive - UWE Bristol/Apps/semt0043/RoboticsScienceAndSystems/StampC3_Template/StampC3_Template.ino"
#include <Wire.h>

#include "RobotWifiAP.h"


RobotWifiAP_c server("myAP", "myPassword", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"



Robot_c robot;

RobotIMU_c imu;

Controller_c controller;
void setup();
void loop();
#line 22 "C:/Users/c85-lau/OneDrive - UWE Bristol/Apps/semt0043/RoboticsScienceAndSystems/StampC3_Template/StampC3_Template.ino"
void setup() {

  Serial.begin(115200);
  server.begin();

  robot.initialise();





  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 );
  }

}



void loop() {


  server.update();




  controller.update(robot, server);

}