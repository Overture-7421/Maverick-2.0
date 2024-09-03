// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "NearShoot.h"
#include "Subsystems/Shooter/Constants.h"

NearShoot::NearShoot(SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void NearShoot::Initialize() {
  superStructure->setToAngle(-31_deg, 50_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterNearShoot);
}

// Called repeatedly when this Command is scheduled to run
void NearShoot::Execute() {}

// Called once the command ends or is interrupted.
void NearShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool NearShoot::IsFinished() {
  if(superStructure->getTargetPosition(-31_deg, 50_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterNearShoot)){
    return true;
  } else {
    return false;
  }

}
