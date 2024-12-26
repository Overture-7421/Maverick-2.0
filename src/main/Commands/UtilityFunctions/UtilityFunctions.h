// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Chassis/Chassis.h"
#include <frc/DriverStation.h>

bool isRedAlliance();
units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose);