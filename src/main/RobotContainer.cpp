// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <fmt/core.h>

RobotContainer::RobotContainer() {

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

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();

	chassis.setAcceptingVisionMeasurements(true);
}

void RobotContainer::ConfigDriverBindings() {
	chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());

	driver.Back().OnTrue(ResetHeading(&chassis));
}

//chassis.enableSpeedHelper(&headingSpeedsHelper);

void RobotContainer::ConfigOperatorBindings() {
}

void RobotContainer::ConfigDefaultCommands() {
	leds.SetDefaultCommand(BlinkEffect(&leds, "all", { 224, 42, 266 }, 4_s).ToPtr().IgnoringDisable(true));

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

void RobotContainer::Simulation(){
	chassis.simMode();
	chassis.simPigeon();
}

void RobotContainer::UpdateTelemetry() {
	frc::SmartDashboard::PutNumber("Pigeon", chassis.getRotation2d().Degrees().value());
	chassis.shuffleboardPeriodic();
}