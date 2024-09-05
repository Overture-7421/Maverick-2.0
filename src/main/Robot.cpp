// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/NamedCommands.h>

void Robot::RobotInit() {
  pathplanner::NamedCommands::registerCommand("autoSpeaker", std::move(
    frc2::cmd::Sequence(
      NearShoot(&superStructure, &shooter).ToPtr(),
      storage.startStorage(),
      frc2::cmd::Wait(0.3_s),
      ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr()
    )));

  pathplanner::NamedCommands::registerCommand("GroundGrabLarge", std::move(
    GroundGrabCommand(&intake, &storage, &superStructure).WithTimeout(2.8_s)
  ));

  pathplanner::NamedCommands::registerCommand("GroundGrabSmall", std::move(
    GroundGrabCommand(&intake, &storage, &superStructure).WithTimeout(0.75_s)
  ));

  


  /*
  pathplanner::NamedCommands::registerCommand("AllignToNote", std::move(
    frc2::cmd::Sequence(
      chassis.enableSpeedHelper(&speedHelperNoteTracking),
      speedHelperNoteTracking(&chassis, &noteTrackingCamera),
      frc2::cmd::Wait(0.5_s),
      chassis.disableSpeedHelper()
  )));
  */
  
  gallitoOro = pathplanner::AutoBuilder::buildAuto("GallitoOro");
  autonomousGallito = pathplanner::AutoBuilder::buildAuto("AutonomousGallito");
  
  autoChooser.SetDefaultOption("None", &defaultAuto);
  autoChooser.AddOption("GallitoOro", &gallitoOro);
  autoChooser.AddOption("AutonomousGallito", &autonomousGallito);



  frc::SmartDashboard::PutData("AutoChooser", &autoChooser);  
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //frc::SmartDashboard::PutNumber("lowerAngleTarget", 0.0);
  //frc::SmartDashboard::PutNumber("upperAngleTarget", 90.0);
  //frc::SmartDashboard::PutNumber("setTheVoltage", 0.0);
  // gamepad.B().WhileTrue(superStructure.SysIdQuasistatic(frc2::sysid::kForward));
  // gamepad.A().WhileTrue(superStructure.SysIdQuasistatic(frc2::sysid::kReverse));
  // gamepad.X().WhileTrue(superStructure.SysIdDynamic(frc2::sysid::kForward));
  // gamepad.Y().WhileTrue(superStructure.SysIdDynamic(frc2::sysid::kReverse));

  leds.SetDefaultCommand(BlinkEffect(&leds, "all", {112, 21, 133}, 4_s).ToPtr().IgnoringDisable(true));

  intakeLeds.WhileTrue(BlinkEffect(&leds, "all", {112, 21, 133}, 0.2_s).ToPtr().IgnoringDisable(true));

  isNoteOnSensorLeds.WhileTrue(StaticEffect(&leds, "all", {14, 227, 49}). ToPtr().IgnoringDisable(true));
  
  //Driver
  driver.B().OnTrue(LowPassCommand(&superStructure, &shooter, &chassis, &gamepad).ToPtr());
  driver.B().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

  driver.X().OnTrue(HighPassCommand(&superStructure, &shooter,&chassis, &gamepad).ToPtr());
  driver.X().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

  driver.RightBumper().WhileTrue(VisionSpeakerCommand(&chassis, &superStructure, &shooter, &gamepad, &offsetUpperShoot, &tagLayout).ToPtr());
  driver.RightBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

  gamepad.upDpad().OnTrue(frc2::cmd::RunOnce([&]{
    offsetUpperShoot += -1.5_deg;
  }).AlongWith(BlinkEffect(&leds, "all", {255, 0, 0}, 0.3_s).ToPtr()).WithTimeout(2_s));

   gamepad.downDpad().OnTrue(frc2::cmd::RunOnce([&]{
    offsetUpperShoot += 1.0_deg;
  }).AlongWith(BlinkEffect(&leds, "all", {0, 0, 255}, 0.3_s).ToPtr()).WithTimeout(2_s));

gamepad.leftDpad().OnTrue(frc2::cmd::RunOnce([&]{
    offsetUpperShoot = 0.0_deg;
  }).AlongWith(BlinkEffect(&leds, "all", {255, 255, 255}, 0.3_s).ToPtr()).WithTimeout(2_s));


  chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());

  //Operator
  gamepad.LeftBumper().OnTrue(AmpCommand(&superStructure, &shooter).ToPtr());
  gamepad.LeftBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

  gamepad.RightBumper().OnTrue(ManualSpeakerCommand(&superStructure, &shooter).ToPtr());
  gamepad.RightBumper().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());

  gamepad.RightTrigger().OnTrue(GroundGrabCommand(&intake, &storage, &superStructure).Unless([&]{
      return driver.GetRightBumper() || storage.isNoteOnSensor() || driver.GetBButton() || driver.GetXButton();
    
  }));


   gamepad.RightTrigger().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr().Unless([&]{
       return driver.GetRightBumper() || driver.GetBButton() || driver.GetXButton();
    
   }));


  gamepad.B().OnTrue(SpitNoteCommand(&intake, &storage, &superStructure));
  gamepad.B().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());
  
  gamepad.LeftTrigger().OnTrue(storage.startStorage());
  gamepad.LeftTrigger().OnFalse(storage.stopStorage());

  //Intake sin sensor A()

  //Escalada manual Y() No esta probada
  gamepad.Y().OnTrue(ManualClimbCommand(&superStructure).ToPtr());
  gamepad.Y().OnFalse(ClosedCommand(&superStructure, &shooter, &storage, &intake).ToPtr());
 

  supportArms.setServoAngle(-110_deg);  

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
	  {29, "Offseason 2024/cancoders/upper_cancoder"},
	  {28, "Offseason 2024/cancoders/lower_cancoder"}


		});

	simDutyCycleEncoderManager.Init({});
#endif
}

AprilTags::Config Robot::shooterCameraConfig() {
    AprilTags::Config config;
    config.cameraName = "Arducam_OV9281_USB_Camera";
    config.cameraToRobot = { -14.950771_in, 0_m, 14.034697_in,{0_deg, -30_deg, 180_deg}};
    return config;
}


AprilTags::Config Robot::frontRightCameraConfig() {
    AprilTags::Config config;
    config.cameraName = "Arducam_OV2311_USB_Camera";
    config.cameraToRobot = {6.388283_in, -10.648092_in, 8.358231_in, {180_deg, -28.125_deg, -30_deg}};
    return config;
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
  chassis.shuffleboardPeriodic();
  //superStructure.getCurrentAngle(superStructure.lowerCANCoder.GetAbsolutePosition().GetValueAsDouble(), superStructure.upperCANCoder.GetAbsolutePosition().GetValueAsDouble());
  //frc::SmartDashboard::PutNumber("upperMotor position", superStructure.upperMotor.GetPosition().GetValueAsDouble());
  //frc::SmartDashboard::PutNumber("lowerMotor position:", superStructure.lowerRightMotor.GetPosition().GetValueAsDouble());

  //frc::SmartDashboard::PutNumber("actualVelocity", shooter.getVelocityVoltage());

}
  //chassis.enableSpeedHelper(&headingSpeedsHelper);




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
  autonomo = autoChooser.GetSelected();
  autonomo->Schedule();
  chassis.setAcceptingVisionMeasurements(false);
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  if(autonomo != nullptr){
    autonomo->Cancel();
  }

   chassis.setAcceptingVisionMeasurements(true);
  
}

void Robot::TeleopPeriodic() {


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
