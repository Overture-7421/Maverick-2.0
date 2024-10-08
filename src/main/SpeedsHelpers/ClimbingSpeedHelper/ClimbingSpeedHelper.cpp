// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClimbingSpeedHelper.h"

ClimbingSpeedHelper::ClimbingSpeedHelper(Chassis *chassis){
    this->chassis = chassis;
    this->yPIDController.SetIZone(3);
	this->yPIDController.SetTolerance(0.05_m);
    this->xPIDController.SetIZone(3);
	this->xPIDController.SetTolerance(0.05_m);
    this->headingPIDController.SetIZone(3);
	this->headingPIDController.SetTolerance(0.05_deg);
}

void ClimbingSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed){

    units::meter_t xGoal = 2.5_m;
    units::meter_t yGoal = 5_m;
    units::degree_t rotationGoal = 90_deg;

    frc::Pose2d pose = chassis->getEstimatedPose();

    double xOut = xPIDController.Calculate(pose.X(), xGoal);
    double yOut = yPIDController.Calculate(pose.Y(), yGoal);
    double rotationOut = headingPIDController.Calculate(pose.Rotation().Degrees(), rotationGoal);

    inputSpeed.vx = units::meters_per_second_t(xOut);
    inputSpeed.vy = units::meters_per_second_t(yOut);
    inputSpeed.omega = units::radians_per_second_t(rotationOut);
}