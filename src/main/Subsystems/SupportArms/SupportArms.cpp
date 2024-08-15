// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SupportArms.h"

SupportArms::SupportArms() = default;

// This method will be called once per scheduler run
void SupportArms::Periodic() { 

    SetRightServoAngle(0.0); // Where the wanted positions of the servos are put on (ranges from 0.0 to 1.0)
    SetLeftServoAngle(0.0);
}

void SupportArms::SetRightServoAngle(double angle) { //Conditional that allows us to invert the right servo if needed.
    if (isRightServoInverted) {
        angle = 1.0 - angle;
    }
    RightServo.Set(angle);
}

void SupportArms::SetLeftServoAngle(double angle) { //Conditional that allows us to invert the leftt servo if needed.
    if (isLeftServoInverted) {
        angle = 1.0 - angle;
    }
    LeftServo.Set(angle);
}