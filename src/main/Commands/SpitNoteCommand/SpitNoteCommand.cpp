// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitNoteCommand.h"
#include <frc2/command/Commands.h>

frc2::CommandPtr SpitNoteCommand(Intake* intake, Storage* storage, SuperStructure* superStructure){

    return frc2::cmd::Sequence(
        superStructure->setAngle(-31_deg, 72_deg),
        frc2::cmd::WaitUntil([superStructure]{
            return superStructure->getTargetPosition(-31_deg, 72_deg);
        }),
        frc2::cmd::Parallel(
            intake->reverseIntake(),
            storage->reverseStorage()
        )  

    );
}
