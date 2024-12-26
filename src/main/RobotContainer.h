// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <OvertureLib/Robots/OverContainer/OverContainer.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
// Include headers for the simulation managers
#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"
#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#include "OvertureLib/Simulation/SimPigeonManager/SimPigeonManager.h"
#include "Subsystems/Chassis/Chassis.h"
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "frc/DriverStation.h"
#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include "OvertureLib/Math/Utils.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Subsystems/LedsManager/LedsManager.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>

#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "Commands/AligntToNote/AlignToNote.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking/SpeedHelperNoteTracking/SpeedHelperNoteTracking.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking/FieldOrientedNoteTracking/FieldOrientedNoteTracking.h"
#include "Commands/DriveCommand/DriveCommand.h"
#include "Commands/ResetHeading/ResetHeading.h"


class RobotContainer : public OverContainer {
public:

	RobotContainer();

	frc2::Command* GetAutonomousCommand();
	frc2::CommandPtr GetTeleopResetCommand();
	void UpdateTelemetry();

private:
	void ConfigureBindings();
	void ConfigDriverBindings();
	void ConfigOperatorBindings();
	void ConfigDefaultCommands();
	void ConfigCharacterizationBindings();

	//Controls
	OverXboxController driver{ 0,0.20, 0.5 };

	//Subsystems
	Chassis chassis;
	units::degree_t offsetUpperShootRed = 3.0_deg;
	units::degree_t offsetUpperShootRedAuto = 1.0_deg;
	units::degree_t offsetUpperShootBlue = 3.0_deg;
	units::degree_t offsetUpperShootBlueAuto = 1.0_deg;

	LedsManager leds{ 8, 240, {{"all", {0, 239}
	  }} };


#ifndef __FRC_ROBORIO__
	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo);
#else
	frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };
#endif 

	static AprilTags::Config shooterCameraConfig();
	static AprilTags::Config frontRightCameraConfig();

	AprilTags shooterCamera{ &tagLayout, &chassis, shooterCameraConfig() };
	AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, frontRightCameraConfig() };
	photon::PhotonCamera noteTrackingCamera{ "PSEye" };


	SpeedHelperNoteTracking speedHelperNoteTracking{ &chassis, &noteTrackingCamera };

	frc::SendableChooser<frc2::Command*> autoChooser;
};
