// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedPassCommand.h"
#include <frc2/command/Commands.h>
#include "Subsystems/Shooter/Constants.h"

frc2::CommandPtr ClosedPassCommand(SuperStructure* superStructure, Shooter* shooter, Storage* storage, Intake* intake ){
    return frc2::cmd::Sequence(
        superStructure->setAngle(0_deg, 89_deg),
        frc2::cmd::WaitUntil([superStructure] {
            return superStructure->getTargetPosition(0_deg, 89_deg);
        }),
        frc2::cmd::Parallel(
            shooter->setObjectiveVelocityPtr(),
            storage->stopStorage(),
            intake->stopIntake(),
            superStructure->setAngle(-31_deg, 89_deg)
        )
    );

}
