// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveCommand.h"
#include <OvertureLib/Gamepad/Gamepad.h>
#include <cmath>

DriveCommand::DriveCommand(Chassis* chassis, Gamepad* gamepad) : headingSpeedsHelper{headingController, chassis} {
  this->chassis = chassis;
  this->gamepad = gamepad;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({chassis});
}

// Called when the command is initially scheduled.
void DriveCommand::Initialize() {
  auto alliance = frc::DriverStation::GetAlliance();
  if(alliance.value() == frc::DriverStation::Alliance::kRed){
    allianceMulti = -1;
  }
  if(alliance.value() == frc::DriverStation::Alliance::kBlue){
    allianceMulti = 1;
  }

}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() {
  frc::Rotation2d targetAngle{gamepad->getRightStickDirection()}; 

  if(allianceMulti == -1){
    targetAngle = targetAngle.RotateBy({180_deg});
  }



  double squares = sqrt(gamepad->GetRightY() * gamepad->GetRightY() + gamepad->GetRightX() * gamepad->GetRightX());

  if(squares > 0.71){
    if(speedHelperMoved == false){
      speedHelperMoved = true;
      chassis->enableSpeedHelper(&headingSpeedsHelper);
    }

  } else if(speedHelperMoved == true){
    speedHelperMoved = false;
    chassis->disableSpeedHelper();
  }

  headingSpeedsHelper.setTargetAngle(targetAngle);

  auto xSpeed = xInput.Calculate(Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetRawAxis(1), 0.2, 0.5) * chassis->getMaxModuleSpeed());
  auto ySpeed = yInput.Calculate(Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetRawAxis(0), 0.2, 0.5) * chassis->getMaxModuleSpeed());
  auto rotationSpeed = (-gamepad->getTwist() * allianceMulti * 5.0_tps);

  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, chassis->getEstimatedPose().Rotation());
  chassis->setTargetSpeeds(speeds);

}

// Called once the command ends or is interrupted.
void DriveCommand::End(bool interrupted) {
    chassis->disableSpeedHelper();

}

// Returns true when the command should end.
bool DriveCommand::IsFinished() {
  return false;
}
