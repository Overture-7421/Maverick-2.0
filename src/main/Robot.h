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

#include <OvertureLib/Subsystems/LedsManager/LedsManager.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h>
#include <OvertureLib/Subsystems/LedsManager/Effects/StaticEffect/StaticEffect.h>
#include "Subsystems/SupportArms/SupportArms.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <OvertureLib/Robots/OverRobot/OverRobot.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <OvertureLib/Gamepad/Gamepad.h>
#include <frc2/command/button/Trigger.h>
#include "Commands/AligntToNote/AlignToNote.h"

#include "Commands/ManualSpeakerCommand/ManualSpeakerCommand.h"
#include "Commands/AmpCommand/AmpCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/ManualSpeakerCommand/ManualSpeakerCommand.h"
#include "Commands/GroundGrabCommand/GroundGrabCommand.h"
#include "Commands/LowPassCommand/LowPassCommand.h"
#include "Commands/HighPassCommand/HighPassCommand.h"
#include "Commands/ClosedPassCommand/ClosedPassCommand.h"
#include "Commands/SpitNoteCommand/SpitNoteCommand.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking/SpeedHelperNoteTracking/SpeedHelperNoteTracking.h"
#include "SpeedsHelpers/SpeedHelperNoteTracking/FieldOrientedNoteTracking/FieldOrientedNoteTracking.h"
#include "Commands/VisionSpeakerCommand/VisionSpeakerCommand.h"
#include "Commands/NearShoot/NearShoot.h"
#include "Commands/ManualClimeCommand/ManualClimbCommand.h"
#include "Commands/DriveCommand/DriveCommand.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Commands/FarSpeakerCommand/FarSpeakerCommand.h"
#include "Commands/FieldOrientedAlignToNote/FieldOrientedAlignToNote.h"
#include "SpeedsHelpers/ClimbingSpeedHelper/ClimbingSpeedHelper.h"

#include "Autos/SourceAutoRace/SourceAutoRace.h"
#include "Autos/AmpAutoRace/AmpAutoRace.h"

#include "Commands/ClosedCommandAuto/ClosedCommandAuto.h"
#include "Commands/GroundGrabCommandAuto/GroundGrabCommandAuto.h"
#include "Commands/Climbing/Cllimbing.h"


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

  Gamepad gamepad{1, 0.1, 0.1};
  Gamepad driver{0,0.25, 0.5};

  Intake intake;
  Storage storage;
  Shooter shooter;
  SuperStructure superStructure;
  SupportArms supportArms;
  units::degree_t offsetUpperShootRed = 0_deg;
  units::degree_t offsetUpperShootBlue = 0_deg;

  LedsManager leds{8, 240, {{"all", {0, 239}
    }}};
  
  frc2::Trigger intakeLeds{[this] {
    return intake.getVoltage() > 0.0;
  }};

  frc2::Trigger isNoteOnSensorLeds{[this] {return storage.isNoteOnSensor();
  }};


 private:
  Chassis chassis;

  #ifndef __FRC_ROBORIO__
    frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo);
  #else
    frc::AprilTagFieldLayout tagLayout {"/home/lvuser/deploy/tag_layout/7421-field.json"};
  #endif 

  static AprilTags::Config shooterCameraConfig();
  static AprilTags::Config frontRightCameraConfig();

  AprilTags shooterCamera{ &tagLayout, &chassis, shooterCameraConfig()};
  AprilTags frontRightSwerveModuleCamera{ &tagLayout, &chassis, frontRightCameraConfig()};
  photon::PhotonCamera noteTrackingCamera{ "C270_HD_WEBCAM" };



  

  SpeedHelperNoteTracking speedHelperNoteTracking{&chassis, &noteTrackingCamera};
  ClimbingSpeedHelper climbingSpeedHelper{&chassis};


  frc::SendableChooser<frc2::CommandPtr*> autoChooser;

  frc2::CommandPtr gallitoOro = frc2::cmd::None();
  frc2::CommandPtr gallitoOroV2 = frc2::cmd::None();
  frc2::CommandPtr sourceAuto = frc2::cmd::None();
  frc2::CommandPtr ampAuto = frc2::cmd::None();
  frc2::CommandPtr autonomousGallito = frc2::cmd::None();
  frc2::CommandPtr defaultAuto = frc2::cmd::None();

  frc2::CommandPtr* autonomo = nullptr;

  int allianceMulti;
};






