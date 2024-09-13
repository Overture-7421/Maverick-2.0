// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommandAuto.h"
#include "Subsystems/Shooter/Constants.h"
#include "Subsystems/Storage/Constants.h"
#include "Subsystems/Intake/Constants.h"

ClosedCommandAuto::ClosedCommandAuto(SuperStructure* superStructure, Shooter* shooter, Storage* storage, Intake* intake) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->storage = storage;
  this->intake = intake;

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter, storage, intake});
}

// Called when the command is initially scheduled.
void ClosedCommandAuto::Initialize() {
  superStructure->setToAngle(-31_deg, 70_deg);
  //shooter->setObjectiveVelocity(ConstantsSh::ShooterClosedCommand);
  storage->setVoltage(ConstantsSt::stopVoltage);
  intake->setVoltage(ConstantsIn::stopVolts);

}

// Called repeatedly when this Command is scheduled to run
void ClosedCommandAuto::Execute() {}

// Called once the command ends or is interrupted.
void ClosedCommandAuto::End(bool interrupted) {}

// Returns true when the command should end.
bool ClosedCommandAuto::IsFinished() {
    return true;

}