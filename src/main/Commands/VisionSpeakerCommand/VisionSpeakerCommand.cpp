// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionSpeakerCommand.h"
#include "VisionSpeakerConstants.h"
#include <Commands/UtilityFunctions/UtilityFunctions.h>

  VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superstructure, Shooter* shooter, Gamepad* gamePad, units::degree_t* offsetVisionShootRed, units::degree_t* offsetVisionShootBlue, frc::AprilTagFieldLayout* tagLayout) : headingSpeedsHelper({7, 0, 0.5,{1200_deg_per_s, 1200_deg_per_s_sq}}, chassis){
    this->chassis = chassis;
    this->superstructure = superstructure;
    this->shooter = shooter;
    this->gamePad = gamePad;
    this->tagLayout = tagLayout;
    this->offsetVisionShootRed = offsetVisionShootRed;
    this->offsetVisionShootBlue = offsetVisionShootBlue;
    AddRequirements({superstructure, shooter});
  }

  VisionSpeakerCommand::VisionSpeakerCommand(Chassis* chassis, SuperStructure* superstructure, Shooter* shooter, units::degree_t* offsetVisionShootRed, units::degree_t* offsetVisionShootBlue, frc::AprilTagFieldLayout* tagLayout) : headingSpeedsHelper({7, 0, 0.5,{1200_deg_per_s, 1200_deg_per_s_sq}}, chassis){
    this->chassis = chassis;
    this->superstructure = superstructure;
    this->shooter = shooter;
    this->gamePad = nullptr;
    this->tagLayout = tagLayout;
    this->offsetVisionShootRed = offsetVisionShootRed;
    this->offsetVisionShootBlue = offsetVisionShootBlue;
    AddRequirements({superstructure, shooter});
  }


// Called when the command is initially scheduled.
void VisionSpeakerCommand::Initialize() {
    if(isRedAlliance()){
      targetObjective = tagLayout->GetTagPose(4).value().ToPose2d().Translation();
    } else {
      targetObjective = tagLayout->GetTagPose(7).value().ToPose2d().Translation();
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
  units::degree_t targetLower = VisionSpeakerConstants::DistanceToLowerAngle[distance];
  units::degree_t targetUpper = VisionSpeakerConstants::DistanceToUpperAngle[distance];

  if(isRedAlliance()){
    offsetUpdated = targetUpper + *offsetVisionShootRed;
  } else {
    offsetUpdated = targetUpper + *offsetVisionShootBlue;
  }

  double targetVelocity = VisionSpeakerConstants::DistanceToShooter[distance];

  superstructure->setToAngle(targetLower, offsetUpdated);
  shooter->setObjectiveVelocity(targetVelocity);




  units::degree_t angleErrorChassis = units::math::abs(targetAngle.Degrees() - chassis->getEstimatedPose().Rotation().Degrees());
  bool allTargetsSS = superstructure->getTargetPosition(targetLower, targetUpper);
  bool onTargetVel = shooter->getObjectiveVelocity(targetVelocity);

  if(gamePad != nullptr){
    if (angleErrorChassis <= 2.0_deg && allTargetsSS == true && onTargetVel == true ){
      gamePad->SetRumble(frc::GenericHID::kBothRumble, 1);
    } else {
      gamePad->SetRumble(frc::GenericHID::kBothRumble, 0);
    }
  }

}

// Called once the command ends or is interrupted.
void VisionSpeakerCommand::End(bool interrupted) {
  chassis->disableSpeedHelper();
  if(gamePad != nullptr){
    gamePad->SetRumble(frc::GenericHID::kBothRumble, 0);
  }
}

// Returns true when the command should end.
bool VisionSpeakerCommand::IsFinished() {
  return false;
}
