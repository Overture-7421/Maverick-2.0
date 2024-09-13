// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "SpeedsHelpers/SpeedHelperNoteTracking.h"
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SuperStructure/SuperStructure.h" 
#include "OvertureLib/Gamepad/Gamepad.h"
#include "Commands/AligntToNote/AlignToNote.h"


frc2::CommandPtr AutoNote(Intake* intake, Storage* storage, SuperStructure* superStructure, Gamepad* gamepad, AlignToNote* alignToNote);
