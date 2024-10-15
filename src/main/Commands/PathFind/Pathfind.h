// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Chassis/Chassis.h"
#include "SpeedsHelpers/ClimbingSpeedHelper/ClimbingSpeedHelper.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Pathfind
    : public frc2::CommandHelper<frc2::Command, Pathfind> {
 public:
  Pathfind(Chassis *chassis, frc::Pose2d pose2d);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Chassis* chassis;
    frc::Pose2d pose2d;
    ClimbingSpeedHelper climbingSpeedHelper;

};
