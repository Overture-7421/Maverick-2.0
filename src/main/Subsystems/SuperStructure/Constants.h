// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

struct ConstantsSS {

  constexpr static const double LowerGearRatio = 172.8;
  constexpr static const double UpperGearRatio = 144;
  constexpr static const double LowerSensorToMechanism = 1.0;
  constexpr static const double UpperSensorToMechanism = 1.0;
  constexpr static const int UpperCANCoderID = 27;
  constexpr static const int LowerCANCoderID = 28;
  constexpr static const double VoltageRamp = 0.01;
  constexpr static const double StatorCurrentLimit = 150;
  constexpr static const double SupplyCurrentLimit = 40;
  constexpr static const double TriggerThresholdCurrent = 60;
  constexpr static const double TriggerThresholdTime = 1;
  constexpr static const double CruiseVelocity = 1.25;
  constexpr static const double CruiseAcceleration = 5;



};
