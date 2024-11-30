// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() {
	leftMotor.setFollow(25, true);

	rightMotor.setSensorToMechanism(ConstantsSh::ShooterGearRatio);
	rightMotor.setRotorToSensorRatio(ConstantsSh::ShooterMotorToSensor);
}

void Shooter::setVoltage(units::volt_t voltage) {
	rightMotor.SetVoltage(voltage);
}

double Shooter::getVelocityVoltage() {
	return rightMotor.GetVelocity().GetValueAsDouble();
}

void Shooter::setObjectiveVelocity(double velocity) {
	rightMotor.SetControl(velocityOutput.WithVelocity(units::turns_per_second_t{ velocity }).WithEnableFOC(false));
}

frc2::CommandPtr Shooter::setObjectiveVelocityPtr() {
	return this->RunOnce([this] {this->setObjectiveVelocity(ConstantsSh::StopShooterSpeaker);});
}

bool Shooter::getObjectiveVelocity(double velocity) {
	double shooterError = velocity - rightMotor.GetVelocity().GetValueAsDouble();

	if (std::abs(shooterError < 5)) {
		return true;
	} else {
		return false;
	}
}


// This method will be called once per scheduler run
void Shooter::Periodic() {}
