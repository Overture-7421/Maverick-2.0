// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Pathfind.h"


Pathfind::Pathfind(Chassis *chassis, frc::Pose2d pose2d) : climbingSpeedHelper(chassis, pose2d) {
 
 this->chassis = chassis;
 this->pose2d = pose2d;

 AddRequirements({chassis});

}

// Called when the command is initially scheduled.
void Pathfind::Initialize() {
  chassis->enableSpeedHelper(&climbingSpeedHelper); 

}

// Called repeatedly when this Command is scheduled to run
void Pathfind::Execute() {}

// Called once the command ends or is interrupted.
void Pathfind::End(bool interrupted) {
  chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool Pathfind::IsFinished() {
  if(climbingSpeedHelper.atGoal()){
    return true;
  } else {
    return false;
  }
}
