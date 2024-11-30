// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowPassCommand.h"
#include "Subsystems/Shooter/Constants.h"
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <frc/DriverStation.h>
#include "Commands/LowPassCommand/LowerPassConstants.h"
#include <Commands/UtilityFunctions/UtilityFunctions.h>

LowPassCommand::LowPassCommand(SuperStructure* superStructure, Shooter* shooter, Chassis* chassis, OverXboxController* gamePad) : headingSpeedsHelper{ headingController, chassis } {
	this->superStructure = superStructure;
	this->shooter = shooter;
	this->chassis = chassis;
	this->gamePad = gamePad;
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ superStructure, shooter });
}

// Called when the command is initially scheduled.
void LowPassCommand::Initialize() {
	if (isRedAlliance()) {
		targetObjective = pathplanner::FlippingUtil::flipFieldPosition(LowerPassConstants::TargetObjective);
	} else {
		targetObjective = LowerPassConstants::TargetObjective;
	}

	chassis->enableSpeedHelper(&headingSpeedsHelper);

	superStructure->setToAngle(7_deg, 89_deg);
	shooter->setObjectiveVelocity(ConstantsSh::ShooterLowPass);
}

// Called repeatedly when this Command is scheduled to run
void LowPassCommand::Execute() {

	frc::Rotation2d targetAngle{ (chassis->getEstimatedPose().X() - targetObjective.X()).value(), (chassis->getEstimatedPose().Y() - targetObjective.Y()).value() };
	headingSpeedsHelper.setTargetAngle(targetAngle);
	units::degree_t angleError = targetAngle.Degrees() - chassis->getEstimatedPose().Rotation().Degrees();

	if (units::math::abs(angleError) <= 2_deg) {
		gamePad->SetRumble(frc::GenericHID::kBothRumble, 1);
	} else {
		gamePad->SetRumble(frc::GenericHID::kBothRumble, 0);
	}

	frc::SmartDashboard::PutNumber("AngleError", angleError.value());

}

// Called once the command ends or is interrupted.
void LowPassCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();
	gamePad->SetRumble(frc::GenericHID::kBothRumble, 0);
}

//Sacar el error

// Returns true when the command should end.
bool LowPassCommand::IsFinished() {
	if (superStructure->getTargetPosition(7_deg, 89_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterLowPass)) {
		return true;
	} else {
		return false;
	}

}
