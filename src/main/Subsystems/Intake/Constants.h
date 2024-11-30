// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/voltage.h>

class ConstantsIn {
 public:

  constexpr static const OverTalonFXConfig intakeMotorConfig = {
	.MotorId = 20,
	.NeutralMode = ControllerNeutralMode::Coast,
	.Inverted = true,
	.useFOC = false,
	.PIDConfigs = ctre::phoenix6::configs::SlotConfigs(),
	.CurrentLimit = 0_A,
	.StatorCurrentLimit = 0_A,
	.TriggerThreshold = 0_A,
	.TriggerThresholdTime = 0_s,
	.ClosedLoopRampRate = 0_s
  };

  constexpr static const units::volt_t GroundGrabVolts = 6.0_V; //Previous 6
  constexpr static const units::volt_t GroundGrabVoltsAuto = 4.0_V;
  constexpr static const units::volt_t stopVolts = 0.0_V;
  constexpr static const units::volt_t reverseVolts = -6.0_V;
  constexpr static const units::volt_t NoteTrackingIn = 7.0_V;
};