// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowPassCommand.h"
#include "Subsystems/Shooter/Constants.h"

LowPassCommand::LowPassCommand(SuperStructure* superStructure, Shooter* shooter) {
  this->superStructure = superStructure;
  this->shooter = shooter;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter});
}

// Called when the command is initially scheduled.
void LowPassCommand::Initialize() {
  superStructure->setToAngle(5_deg, 89_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterLowPass);
}

// Called repeatedly when this Command is scheduled to run
void LowPassCommand::Execute() {}

// Called once the command ends or is interrupted.
void LowPassCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool LowPassCommand::IsFinished() {
  if(superStructure->getTargetPosition(25_deg, 89_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterLowPass)){
    return true;
  } else {
    return false;
  }

}
