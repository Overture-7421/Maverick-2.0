// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Chassis/Chassis.h"
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class VisionSpeakerCommand
    : public frc2::CommandHelper<frc2::Command, VisionSpeakerCommand> {
 public:
  VisionSpeakerCommand(Chassis* chassis, SuperStructure* superstructure);

  void Initialize() override; 

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  TargetingWhileMoving targetWhileMoving{

{

  {1.66_m, 0.22_s},
  {1.9_m, 0.22_s},
  {2.4_m, 0.25_s},
  {2.9_m, 0.23_s},
  {3.4_m, 0.26_s},
  {3.9_m, 0.26_s},
  {4.4_m, 0.34_s},
  {4.9_m, 0.37_s},
  {5.4_m, 0.38_s},
  {5.9_m, 0.43_s},
  {6.4_m, 0.48_s},
  {6.9_m, 0.51_s},



}

  };

  SuperStructure* superstructure;
  Chassis* chassis;
  HeadingSpeedsHelper headingSpeedsHelper;
  };

