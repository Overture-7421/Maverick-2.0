// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitShoot.h"
#include "Subsystems/Shooter/Constants.h"

SpitShoot::SpitShoot(SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void SpitShoot::Initialize() {
  superStructure->setToAngle(-31_deg, 72_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterSpitShoot);
}

// Called repeatedly when this Command is scheduled to run
void SpitShoot::Execute() {}

// Called once the command ends or is interrupted.
void SpitShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool SpitShoot::IsFinished() {
 if(superStructure->getTargetPosition(-31_deg, 72_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterSpitShoot)){
    return true;
  } else {
    return false;
  }
}
