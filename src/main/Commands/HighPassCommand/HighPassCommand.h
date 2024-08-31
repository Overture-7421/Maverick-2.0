// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Chassis/Chassis.h"

#include <frc/controller/ProfiledPIDController.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class HighPassCommand
    : public frc2::CommandHelper<frc2::Command, HighPassCommand> {
 public:
  HighPassCommand(SuperStructure* superStructure, Shooter* shooter, Chassis* chassis);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  SuperStructure* superStructure;
  Shooter* shooter;
  
  Chassis* chassis;
  

  frc::ProfiledPIDController<units::radian> headingController{
    // PID constants: 
    3, 0.0, 0.0, frc::TrapezoidProfile<units::radian>::Constraints{2_rad_per_s, 2_rad_per_s / 1_s} //Constraints max velocity, max acceleration
  };
  HeadingSpeedsHelper headingSpeedsHelper;

};
