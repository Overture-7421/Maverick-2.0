// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "HighPassCommand.h"
#include "Subsystems/Shooter/Constants.h"

HighPassCommand::HighPassCommand(SuperStructure* superStructure, Shooter* shooter, Chassis* chassis, Gamepad* gamePad) : headingSpeedsHelper{headingController, chassis}{
  this->superStructure = superStructure;
  this->shooter = shooter;
  this->chassis = chassis;
  this->gamePad = gamePad;

  AddRequirements({superStructure, shooter, chassis});
}

// Called when the command is initially scheduled.
void HighPassCommand::Initialize() {
  chassis->enableSpeedHelper(&headingSpeedsHelper);

  superStructure->setToAngle(-15_deg, 60_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterHighPass);
  
}

// Called repeatedly when this Command is scheduled to run
void HighPassCommand::Execute() {

  frc::Rotation2d targetAngle{(chassis->getEstimatedPose().X() - 0.65_m).value(), (chassis->getEstimatedPose().Y() - 7.49_m).value()}; 
  headingSpeedsHelper.setTargetAngle(targetAngle);

  units::degree_t angleError = targetAngle.Degrees() - chassis->getEstimatedPose().Rotation().Degrees();

  if (angleError <= 0.2_deg){
   gamePad->rumbleCommand(1);
  } else {
    gamePad->rumbleCommand(0);
  }


}

// Called once the command ends or is interrupted.
void HighPassCommand::End(bool interrupted) {
  chassis->disableSpeedHelper();
  gamePad->rumbleCommand(0);
}

// Returns true when the command should end.
bool HighPassCommand::IsFinished() {
  if(superStructure->getTargetPosition(-15_deg, 60_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterHighPass)){
    return true;
  } else {
    return false;
  }
}
