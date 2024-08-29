// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Commands.h>

#include <frc2/command/CommandHelper.h>
#include "Subsystems/SuperStructure/SuperStructure.h" // Asegúrate de incluir la clase SuperStructure
#include "Subsystems/Shooter/Shooter.h"

class AmpCommand : public frc2::CommandHelper<frc2::Command, AmpCommand> {
 public:
  explicit AmpCommand(SuperStructure* superstructure, Shooter* shooter);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  SuperStructure* superstructure;
  Shooter* shooter;
};


