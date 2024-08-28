#pragma once
#include <frc2/command/CommandHelper.h>
#include "Subsystems/SuperStructure/SuperStructure.h" // Asegúrate de incluir la clase SuperStructure

class ManualSpeakerCommand : public frc2::CommandHelper<frc2::Command, ManualSpeakerCommand> {
 public:
  explicit ManualSpeakerCommand(SuperStructure* superstructure);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  SuperStructure* superstructure;
};
