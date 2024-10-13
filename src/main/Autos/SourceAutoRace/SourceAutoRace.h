// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Chassis/Chassis.h"
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>

frc2::CommandPtr SourceAutoRace(Storage* storage, Chassis* chassis);