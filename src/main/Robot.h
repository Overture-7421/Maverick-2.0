#pragma once

#include <optional>

#include <frc2/command/CommandPtr.h>

#include <OvertureLib/Robots/OverRobot/OverRobot.h>

#include "RobotContainer.h"


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
  frc2::Command* m_autonomousCommand;
	std::optional<frc2::CommandPtr> m_teleopResetCommand;

	RobotContainer m_container;

};
