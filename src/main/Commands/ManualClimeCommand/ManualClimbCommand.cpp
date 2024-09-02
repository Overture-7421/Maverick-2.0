// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ManualClimbCommand.h"

ManualClimbCommand::ManualClimbCommand(SuperStructure* superstructure){
    this->superstructure = superstructure;

  AddRequirements({superstructure});
}


void ManualClimbCommand::Initialize() {
  superstructure->setToAngle(85_deg, 89_deg);
}


void ManualClimbCommand::Execute() {}

void ManualClimbCommand::End(bool interrupted) {}


bool ManualClimbCommand::IsFinished() {
  if(superstructure->getTargetPosition(85_deg, 89_deg)){
    return true;
  } else {
    return false;
  }
}
