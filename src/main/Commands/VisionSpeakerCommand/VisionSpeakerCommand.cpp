// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"
#include "VisionSpeakerConstants.h"

  VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superstructure, Shooter* shooter) : headingSpeedsHelper({7, 0, 0.5,{1200_deg_per_s, 1200_deg_per_s_sq}}, chassis){
    this->chassis = chassis;
    this->superstructure = superstructure;
    this->shooter = shooter;
    AddRequirements({superstructure, shooter});
  }


// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {

  if(auto alliance = frc::DriverStation::GetAlliance()){
    if(alliance.value() == frc::DriverStation::Alliance::kRed){
      targetObjective = pathplanner::GeometryUtil::flipFieldPosition(VisionSpeakerConstants::TargetObjective);
    }
    if(alliance.value() == frc::DriverStation::Alliance::kBlue){
      targetObjective = VisionSpeakerConstants::TargetObjective;
    }
  }

  targetWhileMoving.setTargetLocation(targetObjective);
  chassis->enableSpeedHelper(&headingSpeedsHelper);
  
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {

  frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
  ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
  frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);

  frc::Rotation2d targetAngle((chassis->getEstimatedPose().X() - movingGoalLocation.X()).value(), (chassis->getEstimatedPose().Y() - movingGoalLocation.Y()).value());
  headingSpeedsHelper.setTargetAngle(targetAngle);

  frc::SmartDashboard::PutNumber("target angle:", targetAngle.Degrees().value());

  units::meter_t distance = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
  
  superstructure->setToAngle(VisionSpeakerConstants::DistanceToLowerAngle[distance], VisionSpeakerConstants::DistanceToUpperAngle[distance]);

  shooter->setObjectiveVelocity(VisionSpeakerConstants::DistanceToShooter[distance]);

}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
  chassis->disableSpeedHelper();

}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
  return false;
}
