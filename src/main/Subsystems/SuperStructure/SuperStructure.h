#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include "Constants.h"
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/ArmFeedforward.h>
#include <units/angular_acceleration.h>


class SuperStructure : public frc2::SubsystemBase {
 public:
  SuperStructure();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  // Commands for moving to specific positions (positions might change)
  void setToAngle(units::degree_t lowerAngle, units::degree_t upperAngle);

  frc2::CommandPtr setAngle(units::degree_t lowerAngle, units::degree_t upperAngle);
  bool getTargetPosition(units::degree_t lowerAngle, units::degree_t upperAngle);

  OverTalonFX lowerLeftMotor{22, ControllerNeutralMode::Brake, true, "rio"};
  OverTalonFX lowerRightMotor{21, ControllerNeutralMode::Brake, false, "rio"};
  OverCANCoder lowerCANCoder{ConstantsSS::LowerCANCoderID, -112.148438_deg, "rio"};
  OverTalonFX upperMotor{23, ControllerNeutralMode::Coast, true, "rio"};
  OverCANCoder upperCANCoder{ConstantsSS::UpperCANCoderID, 183.779297_deg, "rio"};

  void getCurrentAngle(double currentLowerAngle, double currentUpperAngle);

  void Periodic() override; //Does nothing since the command will be called from another side

 private:

  frc2::sysid::SysIdRoutine m_sysIdRoutine{frc2::sysid::Config{1_V / 1_s, 3_V, 30_s, nullptr}, 
  frc2::sysid::Mechanism{
    [this](units::volt_t driveVoltage) {
        lowerRightMotor.SetVoltage(driveVoltage);
    }, 
    [this](frc::sysid::SysIdRoutineLog* log) {
      log->Motor("lowerSS")
          .voltage(lowerRightMotor.GetMotorVoltage().GetValue())
          .position(lowerRightMotor.GetPosition().GetValue())
          .velocity(lowerRightMotor.GetVelocity().GetValue());
    }, this}};

  frc::ArmFeedforward armFeedForward{0.3_V, 0.385_V, 0.44488_V / 1_tps, 6.753_V / 1_tr_per_s_sq };
  frc::ArmFeedforward wristFeedForward{0.7_V, 0.9_V, 0.6_V / 1_tps, 4_V / 1_tr_per_s_sq };
};
//
//2.1234