// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>


class SupportArms : public frc2::SubsystemBase { //Class that extends to the SubsystemBase
 public:

SupportArms(); //Constructor

void Periodic() override; //Method that activates the susbystem

void SetRightServoAngle(double angle); //Method to set the angle of the right servo
void SetLeftServoAngle(double angle);  //Method to set the angle of the right servo

 private:
  frc::Servo RightServo{0}; //Declaration of right servo and when will it be connected
  frc::Servo LeftServo{1}; //Declaration of left servo and when will it be connected

  bool isRightServoInverted = false; //Booleans that may invert one servo
  bool isLeftServoInverted = true;
};