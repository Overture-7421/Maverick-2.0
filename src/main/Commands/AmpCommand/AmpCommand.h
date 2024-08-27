// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#pragma once

#include <frc2/command/CommandHelper.h>
#include "Subsystems/SuperStructure/SuperStructure.h" // Asegúrate de incluir la clase SuperStructure

class AmpCommand : public frc2::CommandHelper<frc2::Command, AmpCommand> {
 public:
  explicit AmpCommand(SuperStructure* superstructure);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  frc2::CommandPtr ampCommand();

 private:
  SuperStructure* superstructure;
};


