// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FieldOriented.h"
/*
frc2::CommandPtr FieldOriented(Chassis* chassis, Gamepad* gamepad){
    return frc2::cmd::Sequence(
        WaitForButton(gamepad, frc::XboxController::Button::kBack),
        frc2::cmd::WaitUntil([gamepad]{
            return gamepad->GetBackButton();
        }),
        chassis->resetHeading()
    );
}
*/