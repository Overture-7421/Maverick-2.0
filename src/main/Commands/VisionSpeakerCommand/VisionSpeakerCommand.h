// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Chassis/Chassis.h"

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
  VisionSpeakerCommand();

  void Initialize() override; 

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  TargetingWhileMoving targetWhileMoving{

{
  {0.1_m, 0.1_s},
  {0.5_m, 0.1_s},
  {1.0_m, 0.1_s},
  {1.5_m, 0.1_s},
  {2.0_m, 0.1_s},
  {2.5_m, 0.1_s},
  {3.0_m, 0.1_s},
  {3.5_m, 0.1_s},
  {4.0_m, 0.1_s},
  {4.5_m, 0.1_s},
}

  };

  SuperStructure* superstructure;
  








  };

