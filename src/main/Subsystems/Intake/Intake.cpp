// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() = default;

void Intake::setVoltage(units::volt_t voltage){
    intakeMotor.SetVoltage(voltage);
}

//frc2::CommandPtr intakeCommand = frc2::CommandPtr::

frc2::CommandPtr Intake::startIntake(){return this->RunOnce([this] {Intake::setVoltage(Constants::groundGrabVolts);});};
frc2::CommandPtr Intake::stopIntake(){return this->RunOnce([this] {Intake::setVoltage(Constants::stopVolts);});};
frc2::CommandPtr Intake::reverseIntake(){return this->RunOnce([this] {Intake::setVoltage(Constants::reverseVolts);});};


// This method will be called once per scheduler run
void Intake::Periodic() {
    
}
