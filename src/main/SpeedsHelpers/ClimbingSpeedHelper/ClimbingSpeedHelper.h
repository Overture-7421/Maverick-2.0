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
  ClimbingSpeedHelper(Chassis *chassis);
  void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
  void getRotation();
  void getX();

  void initialize() override;


 private:
  frc::ProfiledPIDController<units::meters> xPIDController{

        2.0, 0.0, 0.0, {4.5_mps, 2_mps_sq} //Constraints max velocity, max acceleration
    };
  frc::ProfiledPIDController<units::meters> yPIDController{

        2.0, 0.0, 0.0, {4.5_mps, 2_mps_sq} //Constraints max velocity, max acceleration
    };
  frc::ProfiledPIDController<units::degree> headingPIDController{

        4, 0.0, 0.0, {500_deg_per_s, 1000_deg_per_s / 1_s} //Constraints max velocity, max acceleration
     };

	Chassis *chassis;
      frc::Pose2d pose2d;
};
















