// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"
#include "VisionSpeakerConstants.h"

  VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superstructure) : headingSpeedsHelper({7, 0, 0.5,{1000_deg_per_s, 1000_deg_per_s_sq}}, chassis){
    this->chassis = chassis;
    this->superstructure = superstructure;
    AddRequirements({chassis, superstructure});
  }


// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {
  targetWhileMoving.setTargetLocation({0.69_m, 5.56_m});
  chassis->enableSpeedHelper(&headingSpeedsHelper);
  
}

// Called repeatedly when this Command is scheduled to run
void VisionSpeakerCommand::Execute() {

  frc::ChassisSpeeds speed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassis->getCurrentSpeeds(), chassis->getEstimatedPose().Rotation());
  ChassisAccels accel = ChassisAccels::FromRobotRelativeAccels(chassis->getCurrentAccels(), chassis->getEstimatedPose().Rotation());
  frc::Translation2d movingGoalLocation = targetWhileMoving.getMovingTarget(chassis->getEstimatedPose(), speed, accel);

  frc::Rotation2d targetAngle((chassis->getEstimatedPose().X() - movingGoalLocation.X()).value(), (chassis->getEstimatedPose().Y() - movingGoalLocation.Y()).value());
  headingSpeedsHelper.setTargetAngle(targetAngle);

  units::meter_t distance = chassis->getEstimatedPose().Translation().Distance(movingGoalLocation);
  
  superstructure->setToAngle(VisionSpeakerConstants::DistanceToLowerAngle[distance], VisionSpeakerConstants::DistanceToUpperAngle[distance]);



}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
  chassis->disableSpeedHelper();

}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
  return false;
}
