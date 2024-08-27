// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/voltage.h>

class ConstantsIn {
 public:

  constexpr static const units::volt_t GroundGrabVolts = 6.0_V;
  constexpr static const units::volt_t stopVolts = 0.0_V;
  constexpr static const units::volt_t reverseVolts = -6.0_V;
};