// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"

class FieldOrientedNoteTracking : public SpeedsHelper  {
  public:
    FieldOrientedNoteTracking(Chassis *chassis, photon::PhotonCamera *noteTrackingCamera);
      void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
      void initialize() override;

  private: 
    frc::ProfiledPIDController<units::meters> wYPIDController{
      10, 0.0, 0.0, {4.5_mps, 15_mps_sq} 
    };

    frc::ProfiledPIDController<units::meters> wXPIDController{
      10, 0.0, 0.0, {4.5_mps, 15_mps_sq} 
    };

    Chassis *chassis;
    photon::PhotonCamera *noteTrackingCamera;

};
