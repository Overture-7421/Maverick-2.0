// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include "Subsystems/Shooter/Constants.h"
#include "Subsystems/Storage/Constants.h"
#include "Subsystems/Intake/Constants.h"

ClosedCommand::ClosedCommand(SuperStructure* superStructure, Shooter* shooter, Storage* storage, Intake* intake) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->storage = storage;
  this->intake = intake;

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter, storage, intake});
}

// Called when the command is initially scheduled.
void ClosedCommand::Initialize() {
  superStructure->setToAngle(-31_deg, 70_deg);
  //shooter->setObjectiveVelocity(ConstantsSh::StopShooterSpeaker);
  storage->setVoltage(ConstantsSt::stopVoltage);
  intake->setVoltage(ConstantsIn::stopVolts);

}

// Called repeatedly when this Command is scheduled to run
void ClosedCommand::Execute() {}

// Called once the command ends or is interrupted.
void ClosedCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClosedCommand::IsFinished() {
    return true;

}