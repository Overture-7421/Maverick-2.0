// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GroundGrabCommand.h"

frc2::CommandPtr GroundGrabCommand(Intake* intake, Storage* storage, SuperStructure* superStructure){
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
        superStructure->setAngle(-32_deg, 80_deg),
        intake->startIntake(),
        storage->startStorage()
    ),
        frc2::cmd::WaitUntil([storage]{
            return storage->isNoteOnSensor();
            
        }),
        frc2::cmd::Parallel(
        intake->stopIntake(),
        storage->stopStorage()
    )
    );
}