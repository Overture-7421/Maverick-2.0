// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoNote.h"

frc2::CommandPtr AutoNote(Intake* intake, Storage* storage, SuperStructure* superStructure, Gamepad* gamepad, AlignToNote* alignToNote){
 return frc2::cmd::Sequence(
        alignToNote
        frc2::cmd::WaitUntil([superStructure]{
            return superStructure->getTargetPosition(-31_deg, 68_deg);
        })
);