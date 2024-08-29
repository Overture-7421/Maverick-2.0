// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include "Subsystems/Shooter/Constants.h"

ClosedCommand::ClosedCommand(SuperStructure* superStructure, Shooter* shooter, Storage* storage) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->storage = storage;

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter, storage});
}

// Called when the command is initially scheduled.
void ClosedCommand::Initialize() {
  superStructure->setToAngle(-25_deg, 85_deg);
  shooter->setObjectiveVelocity(ConstantsSh::StopShooterSpeaker);
  storage->stopStorage();

}

// Called repeatedly when this Command is scheduled to run
void ClosedCommand::Execute() {}

// Called once the command ends or is interrupted.
void ClosedCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClosedCommand::IsFinished() {
  if(superStructure->getTargetPosition(-25_deg, 85_deg) && shooter->getObjectiveVelocity(ConstantsSh::StopShooterSpeaker)){
    return true;
  } else {
    return false;
  }
}
