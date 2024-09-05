// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>

class ConstantsSh {
 public:
  constexpr static const double ShooterSpeaker = 100.0;
  constexpr static const double ShooterNearShoot = 90;
  constexpr static const double ShooterAmp = 70.0;
  constexpr static const double ShooterLowPass = 150.0;
  constexpr static const double ShooterHighPass = 70.0;
  constexpr static const double StopShooterSpeaker = 0.0;
  constexpr static const units::volt_t SpeakerVolt = 12_V;
  constexpr static const units::volt_t SpeakerStopVolt = 0_V;
  constexpr static const units::volt_t TrapVolts = 6.0_V;

  //Gear Ratio 15/28
  constexpr static const double ShooterMotorToSensor = 1.0;
  constexpr static const double ShooterGearRatio = 15.0/28.0 ;
  
};
