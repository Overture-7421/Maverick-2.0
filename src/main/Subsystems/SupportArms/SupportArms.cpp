// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#include "SupportArms.h"


SupportArms::SupportArms() = default;

// This method will be called once per scheduler run
void SupportArms::setServoAngle(units::degree_t angle) { //Conditional that allows us to invert the right servo if needed.
    double angleToServo = angle.value() * (slope) + 0.5;
    double invertedAngle = angle.value() * (-slope) + 0.5;
    rightServo.Set(angleToServo);
    leftServo.Set(invertedAngle);
}

frc2::CommandPtr SupportArms::servoAngleCommand(units::degree_t angle){
  return this->RunOnce([this, angle] {
    this->setServoAngle(angle);
    });
}

  double SupportArms::getRightServoAngle(){
    return rightServo.GetAngle();
  }
  
  double SupportArms::getLeftServoAngle(){
    return leftServo.GetAngle();
  }
                
void SupportArms::Periodic() { 
}

