// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "Constants.h"


class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  void setVoltage(units::volt_t voltage);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr startIntake();
  frc2::CommandPtr stopIntake();
  frc2::CommandPtr reverseIntake();


 private:
    OverTalonFX intakeMotor{20, ControllerNeutralMode::Coast, false, "OverCANivore"};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
