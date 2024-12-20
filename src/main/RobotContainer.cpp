// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Command.h>

RobotContainer::RobotContainer() {
	pathplanner::NamedCommands::registerCommand("autoSpeaker", std::move(
		frc2::cmd::Sequence(
			NearShoot(&superStructure, &shooter).ToPtr().WithTimeout(0.70_s),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			storage.stopStorage()
		)));

	pathplanner::NamedCommands::registerCommand("lol", std::move(
		frc2::cmd::Sequence(
			NearShootFar(&superStructure, &shooter).ToPtr().WithTimeout(0.95_s),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			storage.stopStorage()
		)));

	pathplanner::NamedCommands::registerCommand("spitShoot", std::move(
		frc2::cmd::Sequence(
			frc2::cmd::Wait(1.8_s),
			SpitShoot(&superStructure, &shooter).ToPtr(),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			storage.stopStorage()
		)));

	pathplanner::NamedCommands::registerCommand("spitLowShoot", std::move(
		frc2::cmd::Sequence(
			SpitShoot(&superStructure, &shooter).ToPtr(),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			storage.stopStorage()
		)));

	pathplanner::NamedCommands::registerCommand("FarSpeaker", std::move(
		frc2::cmd::Sequence(
			VisionSpeakerCommand(&chassis, &superStructure, &shooter, &offsetUpperShootRedAuto, &offsetUpperShootBlueAuto, &tagLayout).ToPtr().WithTimeout(1.4_s),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			ClosedCommandAuto(&superStructure, &shooter, &storage, &intake).ToPtr()
		)));

	pathplanner::NamedCommands::registerCommand("VisionSpeaker", std::move(
		frc2::cmd::Sequence(
			VisionSpeakerCommand(&chassis, &superStructure, &shooter, &offsetUpperShootRedAuto, &offsetUpperShootBlueAuto, &tagLayout).ToPtr().WithTimeout(1.0_s),
			storage.startStorage(),
			frc2::cmd::WaitUntil([&] {return !storage.isNoteOnSensor();}),
			ClosedCommandAuto(&superStructure, &shooter, &storage, &intake).ToPtr()
		)));

	pathplanner::NamedCommands::registerCommand("GroundGrabLarge", std::move(
		GroundGrabCommand(&intake, &storage, &superStructure, &gamepad).WithTimeout(3.5_s) //3.5 //Ayer11
	));

	pathplanner::NamedCommands::registerCommand("GroundGrabLargeAuto", std::move(
		GroundGrabCommandAuto(&intake, &storage, &superStructure, &gamepad).WithTimeout(4.0_s)
	));

	pathplanner::NamedCommands::registerCommand("GroundGrabSmall", std::move(
		GroundGrabCommand(&intake, &storage, &superStructure, &gamepad).WithTimeout(2.6_s) //previous 2.6 //Ayer 5.0
	));

	pathplanner::NamedCommands::registerCommand("GroundGrabSmallSlow", std::move(
		GroundGrabCommandAuto(&intake, &storage, &superStructure, &gamepad).WithTimeout(2.6_s) //previous 2.6 //5.0
	));

	pathplanner::NamedCommands::registerCommand("AlignToNote", std::move(
		FieldOrientedAlignToNote(&chassis, &noteTrackingCamera, &storage).ToPtr()
	));

	pathplanner::NamedCommands::registerCommand("NoVision", std::move(
		frc2::cmd::RunOnce([this] {chassis.setAcceptingVisionMeasurements(false);})
	));

	pathplanner::NamedCommands::registerCommand("YesVision", std::move(
		frc2::cmd::RunOnce([this] {chassis.setAcceptingVisionMeasurements(true);})
	));

	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();


	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);


	ConfigureBindings();
	ConfigDriverBindings();
	ConfigOperatorBindings();
	ConfigDefaultCommands();
	//ConfigCharacterizationBindings();

	shooterCameraConfig();
	frontRightCameraConfig();


	chassis.setAcceptingVisionMeasurements(true);

}

void RobotContainer::ConfigureBindings() {
	intakeLeds.WhileTrue(BlinkEffect(&leds, "all", { 224, 42, 266 }, 0.2_s).ToPtr().IgnoringDisable(true));

	fieldOrientedNoteTrackingLeds.WhileTrue(BlinkEffect(&leds, "all", { 255, 255, 0 }, 0.2_s).ToPtr().IgnoringDisable(true));

	isNoteOnSensorLeds.WhileTrue(StaticEffect(&leds, "all", { 28, 254, 98 }).ToPtr().IgnoringDisable(true));
	isNoteOnSensorLeds.OnTrue(driver.getRumbleCommand(1.0).AndThen(frc2::cmd::Wait(1.0_s)).AndThen(driver.getRumbleCommand(0.0)));

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();

	chassis.setAcceptingVisionMeasurements(true);
}

void RobotContainer::ConfigDriverBindings() {
	chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());

	driver.A().OnTrue(LowPassCommand(&superStructure, &shooter, &chassis, &gamepad).ToPtr());
	driver.A().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	driver.LeftBumper().WhileTrue((FieldOrientedAlignToNote(&chassis, &noteTrackingCamera, &intake, &storage, &superStructure).ToPtr()));
	driver.LeftBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	driver.X().OnTrue(HighPassCommand(&superStructure, &shooter, &chassis, &gamepad).ToPtr());
	driver.X().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	driver.RightBumper().WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &gamepad, &offsetUpperShootRed, &offsetUpperShootBlue, &tagLayout).ToPtr());
	driver.RightBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	//driver.Y().WhileTrue(AutoClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &gamepad));
	//driver.Y().OnFalse(superStructure.setAngle(-10_deg, 80_deg));

	driver.B().OnTrue(SpitNoteCommand(&intake, &storage, &superStructure));
	driver.B().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	driver.Back().OnTrue(ResetHeading(&chassis));

	driver.Y().OnTrue(AutoClimb(&chassis, &superStructure, &supportArms, &storage, &shooter, &gamepad));
	driver.Y().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	//driver.Y().WhileTrue(Pathfind(&chassis, {4_m, 4_m, 0_deg}).ToPtr());

}

//chassis.enableSpeedHelper(&headingSpeedsHelper);

void RobotContainer::ConfigOperatorBindings() {
	gamepad.POVUp().OnTrue(frc2::cmd::RunOnce([&] {
		offsetUpperShootRed += -1.0_deg;
		offsetUpperShootBlue += -1.0_deg;

	}).AlongWith(BlinkEffect(&leds, "all", { 255, 0, 0 }, 0.3_s).ToPtr()).WithTimeout(2_s));

	gamepad.POVDown().OnTrue(frc2::cmd::RunOnce([&] {
		offsetUpperShootRed += 1.0_deg;
		offsetUpperShootBlue += 1.0_deg;
	}).AlongWith(BlinkEffect(&leds, "all", { 0, 0, 255 }, 0.3_s).ToPtr()).WithTimeout(2_s));

	gamepad.POVLeft().OnTrue(frc2::cmd::RunOnce([&] {
		offsetUpperShootRed = 0.0_deg;
		offsetUpperShootBlue = 0.0_deg;
	}).AlongWith(BlinkEffect(&leds, "all", { 255, 255, 255 }, 0.3_s).ToPtr()).WithTimeout(2_s));


	gamepad.LeftBumper().OnTrue(AmpCommand(&superStructure, &shooter).ToPtr());
	gamepad.LeftBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	gamepad.RightBumper().OnTrue(ManualSpeakerCommand(&superStructure, &shooter).ToPtr());
	gamepad.RightBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

	gamepad.RightTrigger().OnTrue(GroundGrabCommand(&intake, &storage, &superStructure, &gamepad).Unless([&] {
		return driver.GetHID().GetRightBumperButton() || storage.isNoteOnSensor() || driver.GetHID().GetBButton() || driver.GetHID().GetXButton();

	}));


	gamepad.RightTrigger().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr().Unless([&] {
		return driver.GetHID().GetRightBumperButton() || driver.GetHID().GetBButton() || driver.GetHID().GetXButton();

	}));

	gamepad.LeftTrigger().OnTrue(storage.startStorage());
	gamepad.LeftTrigger().OnFalse(storage.stopStorage());

	//Intake sin sensor A()

	//Escalada manual Y() No esta probada
	gamepad.Y().WhileTrue(ManualClimbCommand(&superStructure).ToPtr());
	gamepad.Y().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());


	/*gamepad.rightDpad().WhileTrue(superStructure.setAngle(90_deg, 90_deg));
	gamepad.rightDpad().OnFalse(superStructure.setAngle(-10_deg, 80_deg));*/

}

void RobotContainer::ConfigDefaultCommands() {
	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 224, 42, 266 }, 4_s).ToPtr().IgnoringDisable(true));

	supportArms.setServoAngle(-110_deg);
}

AprilTags::Config RobotContainer::shooterCameraConfig() {
	AprilTags::Config config;
	config.cameraName = "Global_Shutter_Camera (1)";
	config.cameraToRobot = { -14.950771_in, 0_m, 14.034697_in,{0_deg, -30_deg, 180_deg} };
	return config;
}

AprilTags::Config RobotContainer::frontRightCameraConfig() {
	AprilTags::Config config;
	config.cameraName = "Arducam_OV2311_USB_Camera";
	config.cameraToRobot = { 6.388283_in, -10.648092_in, 8.358231_in, {180_deg, -28.125_deg, -30_deg} };
	return config;
}

void RobotContainer::ConfigCharacterizationBindings() {}

void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();

	superStructure.getCurrentAngle(superStructure.lowerCANCoder.GetAbsolutePosition().GetValueAsDouble(), superStructure.upperCANCoder.GetAbsolutePosition().GetValueAsDouble());
	frc::SmartDashboard::PutNumber("upperMotor position", superStructure.upperMotor.GetPosition().GetValueAsDouble());
	//frc::SmartDashboard::PutNumber("lowerMotor position:", superStructure.lowerRightMotor.GetPosition().GetValueAsDouble());

	//frc::SmartDashboard::PutNumber("actualVelocity", shooter.getVelocityVoltage());
}