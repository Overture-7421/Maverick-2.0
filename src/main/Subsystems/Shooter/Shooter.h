// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <frc2/command/Commands.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void setVelocityVoltage(double velocity, double feedForward);
  frc2::CommandPtr shooterCommand();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  OverTalonFX rightMotor{25, ControllerNeutralMode::Coast, false, "OverCANivore"};
  OverTalonFX leftMotor{26, ControllerNeutralMode::Coast, false, "OverCANivore"};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
