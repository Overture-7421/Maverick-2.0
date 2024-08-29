// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "HighPassCommand.h"
#include "Subsystems/Shooter/Constants.h"

HighPassCommand::HighPassCommand(SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void HighPassCommand::Initialize() {
  superStructure->setToAngle(0_deg, 30_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterHighPass);
}

// Called repeatedly when this Command is scheduled to run
void HighPassCommand::Execute() {}

// Called once the command ends or is interrupted.
void HighPassCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool HighPassCommand::IsFinished() {
  if(superStructure->getTargetPosition(0_deg, 30_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterHighPass)){
    return true;
  } else {
    return false;
  }
}
