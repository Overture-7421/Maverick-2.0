// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //frc::SmartDashboard::PutNumber("lowerAngleTarget", 0.0);
  //frc::SmartDashboard::PutNumber("upperAngleTarget", 90.0);
  //frc::SmartDashboard::PutNumber("setTheVoltage", 0.0);
  // gamepad.B().WhileTrue(superStructure.SysIdQuasistatic(frc2::sysid::kForward));
  // gamepad.A().WhileTrue(superStructure.SysIdQuasistatic(frc2::sysid::kReverse));
  // gamepad.X().WhileTrue(superStructure.SysIdDynamic(frc2::sysid::kForward));
  // gamepad.Y().WhileTrue(superStructure.SysIdDynamic(frc2::sysid::kReverse));


  driver.A().OnTrue(AmpCommand(&superStructure).ToPtr());
  

  //driver.A().OnTrue(intake.startIntake());
  //driver.A().OnFalse(intake.stopIntake());
  
  driver.B().OnTrue(storage.startStorage());
  driver.B().OnFalse(storage.stopStorage());

  driver.Y().OnTrue(shooter.shooterCommand());
  driver.Y().OnFalse(shooter.stopShooterCommand());


  #ifndef __FRC_ROBORIO__
	simMotorManager.Init({
	  {2, "Offseason 2024/motors/back_right_drive"},
	  {4, "Offseason 2024/motors/back_left_drive"},
	  {6, "Offseason 2024/motors/front_left_drive"},
	  {8, "Offseason 2024/motors/front_right_drive"},

	  {1, "Offseason 2024/motors/back_right_rotation"},
	  {3, "Offseason 2024/motors/back_left_rotation"},
	  {5, "Offseason 2024/motors/front_left_rotation"},
	  {7, "Offseason 2024/motors/front_right_rotation"},

    {21, "Offseason 2024/motors/lower_arm"},
    {23, "Offseason 2024/motors/upper_arm"},

    {20, "Offseason 2024/motors/intake_motor"},
    {24, "Offseason 2024/motors/storage_motor"},
    {25, "Offseason 2024/motors/shooter_motor"}


		});

	simPigeonManager.Init("Offseason 2024/imu");

	simCANCoderManager.Init({
	  {9, "Offseason 2024/cancoders/back_right_cancoder"},
	  {10, "Offseason 2024/cancoders/back_left_cancoder"},
	  {11, "Offseason 2024/cancoders/front_left_cancoder"},
	  {12, "Offseason 2024/cancoders/front_right_cancoder"},

	  {27, "Offseason 2024/cancoders/upper_cancoder"},
	  {28, "Offseason 2024/cancoders/lower_cancoder"}


		});

	simDutyCycleEncoderManager.Init({});
#endif

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  //superStructure.getCurrentAngle(superStructure.lowerCANCoder.GetAbsolutePosition().GetValueAsDouble(), superStructure.upperCANCoder.GetAbsolutePosition().GetValueAsDouble());
  //frc::SmartDashboard::PutNumber("upperMotor position", superStructure.upperMotor.GetPosition().GetValueAsDouble());
  //frc::SmartDashboard::PutNumber("lowerMotor position:", superStructure.lowerRightMotor.GetPosition().GetValueAsDouble());

  //frc::SmartDashboard::PutNumber("actualVelocity", shooter.getVelocityVoltage());

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
   //frc::SmartDashboard::PutNumber("objectiveVelocity", 0);
  //frc::SmartDashboard::PutNumber("intakeVoltage", 0);
  //frc::SmartDashboard::PutNumber("storageVoltage", 0);

}

void Robot::TeleopPeriodic() {
  //superStructure.setAngle(units::degree_t(frc::SmartDashboard::GetNumber("lowerAngleTarget", 0.0)), units::degree_t(frc::SmartDashboard::GetNumber("upperAngleTarget", 0.0))).Schedule();
  //superStructure.setToAngle(units::degree_t(frc::SmartDashboard::GetNumber("lowerAngleTarget", 0.0)), units::degree_t(frc::SmartDashboard::GetNumber("upperAngleTarget", 0.0)));
  //superStructure.lowerRightMotor.setVoltage(units::volt_t(frc::SmartDashboard::GetNumber("setTheVoltage", 0.0)), false);

   /*
  double objectiveVelocity = frc::SmartDashboard::GetNumber("objectiveVelocity", 0);
  shooter.setObjectiveVelocity(objectiveVelocity);

  units::volt_t intakeVoltage = units::volt_t(frc::SmartDashboard::GetNumber("intakeVoltage", 0));
  intake.setVoltage(intakeVoltage);

  units::volt_t storageVoltage = units::volt_t(frc::SmartDashboard::GetNumber("storageVoltage", 0));
  storage.setVoltage(storageVoltage);
  */

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
