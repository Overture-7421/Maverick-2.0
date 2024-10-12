// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ResetHeading.h"

frc2::CommandPtr ResetHeading(Chassis* chassis){
    if(auto alliance = frc::DriverStation::GetAlliance()){
        if(alliance.value() == frc::DriverStation::Alliance::kRed){
            frc2::cmd::RunOnce([chassis] {return chassis->resetHeading(180.0);});
        }
        if(alliance.value() == frc::DriverStation::Alliance::kBlue){
            frc2::cmd::RunOnce([chassis]{return chassis->resetHeading(0.0);});
        }
    }
}
