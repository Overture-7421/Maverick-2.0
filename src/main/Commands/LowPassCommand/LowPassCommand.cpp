// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowPassCommand.h"
#include "Subsystems/Shooter/Constants.h"
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <frc/DriverStation.h>
#include "Commands/LowPassCommand/LowerPassConstants.h"
#include <pathplanner/lib/util/GeometryUtil.h>

LowPassCommand::LowPassCommand(SuperStructure* superStructure, Shooter* shooter, Chassis* chassis) : headingSpeedsHelper{headingController, chassis} {
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->chassis = chassis;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({superStructure, shooter, chassis});
}

// Called when the command is initially scheduled.
void LowPassCommand::Initialize() {

  if(auto alliance = frc::DriverStation::GetAlliance()){
    if(alliance.value() == frc::DriverStation::Alliance::kRed){
      targetObjective = pathplanner::GeometryUtil::flipFieldPosition(LowerPassConstants::TargetObjective);
    }
    if(alliance.value() == frc::DriverStation::Alliance::kBlue){
      targetObjective = LowerPassConstants::TargetObjective;
    }
  }


  chassis->enableSpeedHelper(&headingSpeedsHelper);
 
  superStructure->setToAngle(5_deg, 89_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterLowPass);
}

// Called repeatedly when this Command is scheduled to run
void LowPassCommand::Execute() {
  
  frc::Rotation2d targetAngle{(chassis->getEstimatedPose().X() - targetObjective.X()).value(), (chassis->getEstimatedPose().Y() - targetObjective.Y()).value()}; 
  headingSpeedsHelper.setTargetAngle(targetAngle);
}

// Called once the command ends or is interrupted.
void LowPassCommand::End(bool interrupted) {
  chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool LowPassCommand::IsFinished() {
  if(superStructure->getTargetPosition(5_deg, 89_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterLowPass)){
    return true;
  } else {
    return false;
  }

}
