// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ClimbingSpeedHelper : public SpeedsHelper{
 public:
  ClimbingSpeedHelper(Chassis* chassis, frc::Pose2d targetPose);
  void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
  bool atGoal();

  void initialize() override;


 private:
    frc::ProfiledPIDController<units::meters> yPIDController{
        4, 0.0, 0.0, {4.0_mps, 2.5_mps_sq} //Before PID 10, Before mps 4.5
    };

    frc::ProfiledPIDController<units::meters> xPIDController{
        4, 0.0, 0.0, {4.0_mps, 2.5_mps_sq} //Before PID 7, Before mps 4.5
    };
    frc::ProfiledPIDController<units::degree> headingPIDController{
        4, 0.0, 0.0, {800_deg_per_s, 500_deg_per_s / 1_s} //Constraints max velocity, max acceleration
    };

	Chassis* chassis;
    frc::Pose2d targetPose;
    
};
















