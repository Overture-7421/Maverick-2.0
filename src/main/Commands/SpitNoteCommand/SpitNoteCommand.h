// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SuperStructure/SuperStructure.h"


frc2::CommandPtr SpitNoteCommand(Intake* intake, Storage* storage, SuperStructure* superStructure);

