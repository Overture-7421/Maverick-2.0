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

  void setServoAngle(units::degree_t angle); //Method to set the angle of the right servo

  void setServoAngle(units::degree_t angle); //Method to set the angle of the right servo
  
 private:
  frc::Servo rightServo{0}; //Declaration of right servo and when will it be connected
  frc::Servo leftServo{1}; //Declaration of left servo and when will it be connected

  const double slope = 0.5/135;
  
};