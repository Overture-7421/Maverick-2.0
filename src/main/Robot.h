#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "OvertureLib/Gamepad/Gamepad.h"
// Include headers for the simulation managers
#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"
#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#include "OvertureLib/Simulation/SimPigeonManager/SimPigeonManager.h"
#include "OvertureLib/Robots/OverRobot/OverRobot.h"
#include "Subsystems/Chassis/Chassis.h"
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "frc/DriverStation.h"
#include "OvertureLib/Gamepad/Gamepad.h"
#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include "OvertureLib/Math/Utils.h"

class Robot : public OverRobot {
 public:

 
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

  frc::Joystick joystick{0};
  Gamepad gamepad{0,0.2, 0.1}; 
  Chassis chassis;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField
(frc::AprilTagField::k2024Crescendo);

  static AprilTags::Config shooterCameraConfig();
  static AprilTags::Config frontRightCameraConfig();

  AprilTags shooterCamera{ &tagLayout, &chassis, shooterCameraConfig()};
  AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, frontRightCameraConfig()};
  

  /*AprilTags::Config Camera1{camera1Config()};
  AprilTags::Config Camera2{camera2Config()};*/
  
};
