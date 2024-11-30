// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>

class ConstantsSh {
public:
	constexpr static const double ShooterSpeaker = 100.0;
	constexpr static const double ShooterNearShoot = 115.0;
	constexpr static const double ShooterNearShootFar = 100.0;
	constexpr static const double ShooterSpitShoot = 25.0; //40
	constexpr static const double ShooterAmp = 70.0;
	constexpr static const double ShooterLowPass = 150.0;
	constexpr static const double ShooterHighPass = 75.0;
	constexpr static const double ShooterFarSpeaker = 120;
	constexpr static const double StopShooterSpeaker = 0.0;
	constexpr static const double ShooterClosedCommand = 30.0;
	constexpr static const units::volt_t SpeakerVolt = 12_V;
	constexpr static const units::volt_t SpeakerStopVolt = 0_V;
	constexpr static const units::volt_t TrapVolts = 6.0_V;

	//Gear Ratio 15/28
	constexpr static const double ShooterMotorToSensor = 1.0;
	constexpr static const double ShooterGearRatio = 15.0 / 28.0;


	constexpr static const OverTalonFXConfig rightMotorConfig{
	  .MotorId = 25,
	  .NeutralMode = ControllerNeutralMode::Coast,
	  .Inverted = false,
	  .useFOC = false,
	  .PIDConfigs = ctre::phoenix6::configs::SlotConfigs().WithKP(0.1).WithKV(0.065),
	  .CurrentLimit = 40_A,
	  .StatorCurrentLimit = 0_A,
	  .TriggerThreshold = 60_A,
	  .TriggerThresholdTime = 0.25_s,
	  .ClosedLoopRampRate = 0.1_s,
	  .OpenLoopRampRate = 0_s
	};

	constexpr static const OverTalonFXConfig leftMotorConfig{
	  .MotorId = 26,
	  .NeutralMode = ControllerNeutralMode::Coast,
	  .Inverted = false,
	  .useFOC = false,
	  .PIDConfigs = ctre::phoenix6::configs::SlotConfigs(),
	  .CurrentLimit = 40_A,
	  .StatorCurrentLimit = 0_A,
	  .TriggerThreshold = 60_A,
	  .TriggerThresholdTime = 0.25_s,
	  .ClosedLoopRampRate = 0.1_s,
	  .OpenLoopRampRate = 0_s
	};
};
