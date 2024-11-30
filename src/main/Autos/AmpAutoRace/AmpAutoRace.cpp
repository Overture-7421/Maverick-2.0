// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AmpAutoRace.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <Commands/UtilityFunctions/UtilityFunctions.h>

frc2::CommandPtr AmpAutoRace(Storage* storage, Chassis* chassis) {
	frc::Pose2d startingPose = pathplanner::PathPlannerPath::fromPathFile("AmpAuto1").get()->getStartingHolonomicPose().value();

	if (isRedAlliance()) {
		startingPose = pathplanner::FlippingUtil::flipFieldPose(startingPose);
	}
	return frc2::cmd::Sequence(
		frc2::cmd::Parallel(
			pathplanner::NamedCommands::getCommand("autoSpeaker"),
			frc2::cmd::RunOnce([=]() {chassis->resetOdometry(startingPose);})
		),
		frc2::cmd::Parallel(
			pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto1")),
			pathplanner::NamedCommands::getCommand("GroundGrabLarge")
		),
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Sequence(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto2")),
					pathplanner::NamedCommands::getCommand("FarSpeaker")
				),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto3")),
					pathplanner::NamedCommands::getCommand("GroundGrabLarge")
				)
			),
			frc2::cmd::Parallel(
				pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto-Alt-1")),
				pathplanner::NamedCommands::getCommand("GroundGrabSmall")
			),
			[=] {return storage->isNoteOnSensor();}
		),
		frc2::cmd::Either(
			frc2::cmd::Sequence(
				frc2::cmd::Sequence(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto4")),
					pathplanner::NamedCommands::getCommand("FarSpeaker")
				),
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto5")),
					pathplanner::NamedCommands::getCommand("GroundGrabLargeAuto")
				),
				frc2::cmd::Sequence(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto-Alt-3")),
					pathplanner::NamedCommands::getCommand("FarSpeaker")
				)
			),
			frc2::cmd::Sequence(
				frc2::cmd::Parallel(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto-Alt-2")),
					pathplanner::NamedCommands::getCommand("GroundGrabSmall")
				),
				frc2::cmd::Sequence(
					pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("AmpAuto-Alt-3")),
					pathplanner::NamedCommands::getCommand("FarSpeaker")
				)
			),
			[=] {return storage->isNoteOnSensor();}
		)
	);
}
