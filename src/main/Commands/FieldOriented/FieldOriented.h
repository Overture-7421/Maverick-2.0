// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Chassis/Chassis.h"
#include "OvertureLib/Gamepad/Gamepad.h"

#include <frc2/command/Commands.h>
#include "Commands/WaitForButton/WaitForButton.h"

frc2::CommandPtr FieldOriented(Chassis* chassis, Gamepad* gamepad);