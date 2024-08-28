// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AmpCommand.h"

AmpCommand::AmpCommand(SuperStructure superstructure, Shooter shooter) {
  this->superStructure = superStructure;
  AddRequirements(superStructure, shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AmpCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AmpCommand::Execute() {}

// Called once the command ends or is interrupted.
void AmpCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpCommand::IsFinished() {
  return false;
}
