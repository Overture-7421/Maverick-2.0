// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Storage.h"

class ConstantsSt {

  public:

  constexpr static const units::volt_t startVoltage = 4_V; //Previous 4
  constexpr static const units::volt_t startVoltageAuto = 2_V;
  constexpr static const units::volt_t stopVoltage = 0_V;
  constexpr static const units::volt_t reverseVoltage = -4_V;
  constexpr static const units::volt_t NoteTrackingSt = 5.0_V;


};
