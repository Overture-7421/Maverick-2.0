/*#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <OvertureLib/Gamepad/Gamepad.h>
#include "Subsystems/Chassis/Chassis.h"
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h"




class SwerveDriveCommand : public frc2::CommandHelper<frc2::CommandBase, SwerveDriveCommand> {
 public:
  SwerveDriveCommand(Chassis* chassis, Gamepad* gamepad, HeadingSpeedsHelper* headingHelper);

  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  Chassis* chassis;
  Gamepad* gamepad;
  HeadingSpeedsHelper* headingHelper;
};
*/