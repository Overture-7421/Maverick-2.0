// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking/FieldOrientedNoteTracking/FieldOrientedNoteTracking.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FieldOrientedAlignToNote
    : public frc2::CommandHelper<frc2::Command, FieldOrientedAlignToNote> {
 public:
  FieldOrientedAlignToNote(Chassis *chassis, photon::PhotonCamera *noteTrackingCamera, Intake *intake, Storage *storage, SuperStructure *superStructure);
  FieldOrientedAlignToNote(Chassis *chassis, photon::PhotonCamera *noteTrackingCamera, Storage *storage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  FieldOrientedNoteTracking noteTracking;
  Chassis* chassis = nullptr;
  Intake* intake = nullptr;
  Storage* storage = nullptr;
  SuperStructure* superStructure = nullptr;
  photon::PhotonCamera* camera = nullptr;
};
