// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClimbingSpeedHelper.h"
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/pathfinding/Pathfinder.h>

ClimbingSpeedHelper::ClimbingSpeedHelper(Chassis* chassis, frc::Pose2d pose2d){
    this->chassis = chassis;
    this->pose2d = pose2d;
    this->yPIDController.SetIZone(3);
	this->yPIDController.SetTolerance(0.15_m);
    this->xPIDController.SetIZone(3);
	this->xPIDController.SetTolerance(0.15_m);
    this->headingPIDController.SetIZone(3);
	this->headingPIDController.SetTolerance(3.0_deg);
    headingPIDController.EnableContinuousInput(-180_deg, 180_deg);

    if(auto alliance = frc::DriverStation::GetAlliance()){
    if(alliance.value() == frc::DriverStation::Alliance::kRed){
      pose2d = pathplanner::GeometryUtil::flipFieldPose(pose2d);
    } else if(alliance.value() == frc::DriverStation::Alliance::kBlue){
      pose2d = pose2d;
    }
  }

}

void ClimbingSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed){


    frc::Pose2d pose = chassis->getEstimatedPose();

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(pose.X(), pose2d.X()));
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(pose.Y(), pose2d.Y()));
    auto rotationOut = units::degrees_per_second_t(headingPIDController.Calculate(pose.Rotation().Degrees(), pose2d.Rotation().Degrees()));

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut, yOut, rotationOut, chassis->getEstimatedPose().Rotation());

    if (headingPIDController.AtGoal()) {
		rotationOut = 0_deg_per_s;
	}

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    frc::SmartDashboard::PutNumber("TRAP/Rotation: ", pose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("TRAP/X: ", pose.X().value());
    frc::SmartDashboard::PutNumber("TRAP/Y: ", pose.Y().value());

    inputSpeed = speeds;

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

bool ClimbingSpeedHelper::atGoal(){
    if (headingPIDController.AtGoal() && xPIDController.AtGoal() && yPIDController.AtGoal()){
        return true;
    } 
}