// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GroundGrabCommand.h"

frc2::CommandPtr GroundGrabCommand(Intake* intake, Storage* storage){
    return frc2::cmd::Parallel(
        intake->startIntake(),
        storage->startStorage()
    );
};