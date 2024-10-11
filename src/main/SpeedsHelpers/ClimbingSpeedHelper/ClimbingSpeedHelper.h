// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>

class ClimbingSpeedHelper : public SpeedsHelper{
 public:
  ClimbingSpeedHelper(Chassis *chassis);
  void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;



 private:
  frc::ProfiledPIDController<units::meters> xPIDController{

        0.0, 0.0, 0.0, {4.5_mps, 15_mps_sq} //Constraints max velocity, max acceleration
    };
  frc::ProfiledPIDController<units::meters> yPIDController{

        0.0, 0.0, 0.0, {4.5_mps, 15_mps_sq} //Constraints max velocity, max acceleration
    };
  frc::ProfiledPIDController<units::degree> headingPIDController{

        5, 0.0, 0.0, {1000_deg_per_s, 850_deg_per_s / 1_s} //Constraints max velocity, max acceleration
     };

	Chassis *chassis;
      frc::Pose2d pose2d;
};
















