#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>


class SuperStructure : public frc2::SubsystemBase {
 public:
 
  /*
  SuperStructure();

  // Commands for moving to specific positions (positions might change)
  frc2::CommandPtr setAngle(units::degree_t lowerAngle, units::degree_t upperAngle);

  void Periodic() override; //Does nothing since the command will be called from another side

 private:
  */
  /* Motors, and how they must be declared (First number is ID, Second thing is the status of the motor, boolean is 
  if it is inverted) */

  /*
  OverTalonFX lowerLeftMotor{10, ControllerNeutralMode::Coast, false, "rio"};
  OverTalonFX lowerRightMotor{11, ControllerNeutralMode::Coast, false, "rio"};
  OverTalonFX upperMotor{12, ControllerNeutralMode::Brake, false, "rio"};

  */
};

