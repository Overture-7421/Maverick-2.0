// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"
#include "Constants.h"

Shooter::Shooter() = default;


void Shooter::setVelocityVoltage(double velocity, double feedForward){
    leftMotor.setFollow(25, true);
    rightMotor.setVelocityVoltage(velocity, feedForward, false);
}

frc2::CommandPtr Shooter::shooterCommand(){
    return this->RunOnce([this] {Shooter::setVelocityVoltage(Constants::ShooterSpeaker, 0);});
};

// This method will be called once per scheduler run
void Shooter::Periodic() {}
