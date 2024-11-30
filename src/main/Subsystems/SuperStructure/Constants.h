// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <OvertureLib/MotorControllers/OverTalonFX/Config.h>
#include <OvertureLib/Sensors/OverCANCoder/Config.h>

struct ConstantsSS {

	constexpr static const double LowerGearRatio = 172.8;
	constexpr static const double UpperGearRatio = 144;
	constexpr static const double LowerSensorToMechanism = 1.0;
	constexpr static const double UpperSensorToMechanism = 1.0;
	constexpr static const int UpperCANCoderID = 27;
	constexpr static const int LowerCANCoderID = 28;
	constexpr static const units::second_t VoltageRamp = 0.01_s;
	constexpr static const units::ampere_t StatorCurrentLimit = 150_A;
	constexpr static const units::ampere_t SupplyCurrentLimit = 40_A;
	constexpr static const units::ampere_t TriggerThresholdCurrent = 60_A;
	constexpr static const units::second_t TriggerThresholdTime = 1_s;
	constexpr static const units::turns_per_second_t LowerCruiseVelocity = 1.5_tps; // 1.25
	constexpr static const units::turns_per_second_squared_t LowerCruiseAcceleration = 7_tr_per_s_sq; //5
	constexpr static const units::turns_per_second_t UpperCruiseVelocity = 5.0_tps; //2.0
	constexpr static const units::turns_per_second_squared_t UpperCruiseAcceleration = 15_tr_per_s_sq; //5.2

	constexpr static const OverTalonFXConfig lowerLeftConfig = {
		22,
		ControllerNeutralMode::Brake,
		true,
		true,
		ctre::phoenix6::configs::SlotConfigs(),
		SupplyCurrentLimit,
		StatorCurrentLimit,
		TriggerThresholdCurrent,
		TriggerThresholdTime,
		VoltageRamp,
		VoltageRamp

	};

	constexpr static const OverTalonFXConfig lowerRightConfig = {
		21,
		ControllerNeutralMode::Brake,
		false,
		true,
		ctre::phoenix6::configs::SlotConfigs().WithKP(100).WithKI(10),
		SupplyCurrentLimit,
		StatorCurrentLimit,
		TriggerThresholdCurrent,
		TriggerThresholdTime,
		VoltageRamp,
		VoltageRamp
	};

	constexpr static const CanCoderConfig lowerCANCoderConfig = {
		LowerCANCoderID,
		SensorDirectionValue::Clockwise_Positive,
		-112.148438_deg
	};

	constexpr static const OverTalonFXConfig upperConfig = {
		23,
		ControllerNeutralMode::Brake,
		true,
		true,
		ctre::phoenix6::configs::SlotConfigs().WithKP(100).WithKI(60),
		SupplyCurrentLimit,
		StatorCurrentLimit,
		TriggerThresholdCurrent,
		TriggerThresholdTime,
		VoltageRamp,
		VoltageRamp
	};


	constexpr static const CanCoderConfig upperCANCoderConfig = {
		UpperCANCoderID,
		SensorDirectionValue::Clockwise_Positive,
		183.779297_deg
	};
};
