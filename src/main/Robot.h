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
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>
#include <OvertureLib/Subsystems/LedsManager/LedsManager.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>

#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include <OvertureLib/Gamepad/Gamepad.h>

#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/SupportArms/SupportArms.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <OvertureLib/Gamepad/Gamepad.h>
#include <frc2/command/button/Trigger.h>

#include "Commands/ManualSpeakerCommand/ManualSpeakerCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/ManualSpeakerCommand/ManualSpeakerCommand.h"
#include "Commands/GroundGrabCommand/GroundGrabCommand.h"
#include "Commands/LowPassCommand/LowPassCommand.h"
#include "Commands/HighPassCommand/HighPassCommand.h"
#include "Commands/ClosedPassCommand/ClosedPassCommand.h"
#include "Commands/SpitNoteCommand/SpitNoteCommand.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking.h"
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"

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
  void AlignToNote();

  Gamepad gamepad{1, 0.1, 0.1};
  Gamepad driver{0,0.25, 0.5};

  Intake intake;
  Storage storage;
  Shooter shooter;
  SuperStructure superStructure;
  SupportArms supportArms;
  LedsManager leds{8, 240, {
    {"all", {0, 239}}
  }};

  frc2::Trigger intakeLeds{[this] {
     return intake.getVoltage() > 0.0;
      }};

  frc2::Trigger isNoteOnSensorLeds{[this] {return storage.isNoteOnSensor();
  }};

  


 private:
  //Gamepad operator{1,0.2, 0.1}; 

  Chassis chassis;

  frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField
  (frc::AprilTagField::k2024Crescendo);

  static AprilTags::Config shooterCameraConfig();
  static AprilTags::Config frontRightCameraConfig();

  AprilTags shooterCamera{ &tagLayout, &chassis, shooterCameraConfig()};
  AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, frontRightCameraConfig()};
  photon::PhotonCamera noteTrackingCamera{ "PSEye" };
  frc::SlewRateLimiter<units::meters_per_second> xInput{10_mps_sq};
  frc::SlewRateLimiter<units::meters_per_second> yInput{10_mps_sq};
  frc::SlewRateLimiter<units::radians_per_second> wInput{12.5664_rad_per_s_sq};


  SpeedHelperNoteTracking speedHelperNoteTracking{&chassis, &noteTrackingCamera};

  frc::SendableChooser<frc2::CommandPtr*> autoChooser;


  frc2::CommandPtr gallitoOro = frc2::cmd::None();
  frc2::CommandPtr defaultAuto = frc2::cmd::None();

  frc2::CommandPtr* autonomo = nullptr;
};






