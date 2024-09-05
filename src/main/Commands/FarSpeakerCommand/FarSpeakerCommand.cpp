// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FarSpeakerCommand.h"
#include "Subsystems/Shooter/Constants.h"

FarSpeakerCommand::FarSpeakerCommand(SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void FarSpeakerCommand::Initialize() {
  superStructure->setToAngle(-12_deg, 72.5_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterFarSpeaker);
}

// Called repeatedly when this Command is scheduled to run
void FarSpeakerCommand::Execute() {}

// Called once the command ends or is interrupted.
void FarSpeakerCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool FarSpeakerCommand::IsFinished() {
  if(superStructure->getTargetPosition(-12_deg, 72.5_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterFarSpeaker)){
    return true;
  } else {
    return false;
  }
}
