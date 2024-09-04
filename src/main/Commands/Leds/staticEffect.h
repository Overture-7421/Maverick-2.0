// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class staticEffect
    : public frc2::CommandHelper<frc2::Command, staticEffect> {
 public:
  staticEffect();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
