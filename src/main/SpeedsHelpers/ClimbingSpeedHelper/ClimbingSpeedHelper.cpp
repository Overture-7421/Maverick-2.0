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
    headingPIDController.EnableContinuousInput(-180_deg, 180_deg);

}

void ClimbingSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed){

    units::meter_t xGoal = 1_m;
    units::meter_t yGoal = 1_m;
    units::degree_t rotationGoal = 75_deg;

    frc::Pose2d pose = chassis->getEstimatedPose();

    double xOut = xPIDController.Calculate(pose.X(), xGoal);
    double yOut = yPIDController.Calculate(pose.Y(), yGoal);
    double rotationOut = headingPIDController.Calculate(pose.Rotation().Degrees(), rotationGoal);

   
    

    if (headingPIDController.AtSetpoint()) {
		rotationOut = 0;
	}

    if (xPIDController.AtSetpoint()) {
        xOut = 0;
    }

    if (yPIDController.AtSetpoint()) {
        yOut = 0;
    }

    inputSpeed.omega = units::degrees_per_second_t(rotationOut);
    inputSpeed.vx = units::meters_per_second_t(xOut);
    inputSpeed.vy = units::meters_per_second_t(0);


}

void ClimbingSpeedHelper::getRotation(){
    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::SmartDashboard::PutNumber("Rotation: ", pose.Rotation().Degrees().value());
}

void ClimbingSpeedHelper::getX(){
    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::SmartDashboard::PutNumber("X: ", pose.X().value());
}

void ClimbingSpeedHelper::initialize(){
    headingPIDController.Reset(chassis->getEstimatedPose().Rotation().Radians());

}