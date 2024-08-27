// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include <OvertureLib/Gamepad/Gamepad.h>

#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Shooter/Shooter.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <OvertureLib/Gamepad/Gamepad.h>
#include "Commands/ManualSpeakerCommand/ManualSpeakerCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"

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

  Gamepad gamepad{0, 0.1, 0.1};
  Gamepad driver{0,0.25, 0.5};

  Intake intake;
  Storage storage;
  Shooter shooter;
  SuperStructure superStructure;



 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

   ManualSpeakerCommand* manualSpeakerCommand;
   AmpCommand ampCommand;

  


};
