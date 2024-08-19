#pragma once
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

class Chassis : public SwerveChassis {
 
 public:
  Chassis();

  void shuffleboardPeriodic();



 private:

  frc::Field2d field2d;
 
  static ModuleConfig FrontLeftConfig();
  static ModuleConfig FrontRightConfig();
  static ModuleConfig BackLeftConfig();
  static ModuleConfig BackRightConfig();
 
  static frc::SimpleMotorFeedforward<units::meters> feedForwardFrontLeft;
  static frc::SimpleMotorFeedforward<units::meters> feedForwardFrontRight;
  static frc::SimpleMotorFeedforward<units::meters> feedForwardBackLeft;
  static frc::SimpleMotorFeedforward<units::meters> feedForwardBackRight;

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;
};
