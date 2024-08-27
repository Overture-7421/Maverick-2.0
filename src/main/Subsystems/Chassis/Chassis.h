#pragma once
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Translation2d.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"



class Chassis : public SwerveChassis {
public:
    Chassis();
 
  void shuffleboardPeriodic();
  void Drive(const frc::ChassisSpeeds& speeds); // Method to control the chassisprivate:
   
  units::meters_per_second_t getMaxModuleSpeed() override; 
  units::meter_t getDriveBaseRadius() override;
  frc::Rotation2d getRotation2d() override;
  frc::Rotation3d getRotation3d() override;
  void getTo();

  SwerveModule& getFrontLeftModule() override;
  SwerveModule& getFrontRightModule() override;
  SwerveModule& getBackLeftModule() override;
  SwerveModule& getBackRightModule() override;
     
  frc::SlewRateLimiter<units::meters_per_second>& getVxLimiter() override;
	frc::SlewRateLimiter<units::meters_per_second>& getVyLimiter() override;
	frc::SlewRateLimiter<units::radians_per_second>& getVwLimiter() override;

  frc::SwerveDriveKinematics<4>& getKinematics() override;

  wpi::log::StructLogEntry<frc::Pose2d>& getPoseLog() override;
  wpi::log::StructLogEntry<frc::Pose2d>& getVisionPoseLog() override;

 
private:
  OverPigeon pigeon{13, "OverCANivore"}; 

  // Module configurations
  static ModuleConfig FrontLeftConfig();
  static ModuleConfig FrontRightConfig();
  static ModuleConfig BackLeftConfig();
  static ModuleConfig BackRightConfig();

  // Swerve modules
  SwerveModule frontLeftModule{Chassis::FrontLeftConfig()};
  SwerveModule frontRightModule{Chassis::FrontRightConfig()};
  SwerveModule backLeftModule{Chassis::BackLeftConfig()};
  SwerveModule backRightModule{Chassis::BackRightConfig()};

 //SLEW RATE LIMITERS :)
  frc::SlewRateLimiter<units::meters_per_second> vxLimiter{18_mps_sq};
  frc::SlewRateLimiter<units::meters_per_second> vyLimiter{18_mps_sq};
  frc::SlewRateLimiter<units::radians_per_second> vwLimiter{12.5664_rad_per_s_sq};

//POSE LOGS
 wpi::log::DataLog& log = frc::DataLogManager::GetLog();
 wpi::log::StructLogEntry<frc::Pose2d> poseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/chassis/pose");
 wpi::log::StructLogEntry<frc::Pose2d> visionPoseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/chassis/vision_pose");


  // Kinematics for chassis configuration
  frc::Field2d field2d;
  frc::ChassisSpeeds desiredSpeeds;
  ChassisAccels currentAccels;
  frc::Pose2d latestPose;
  frc::SwerveDriveKinematics<4> kinematics {{
                 frc::Translation2d {-13.125_in, -10.375_in}, //BackLeftModule
                 frc::Translation2d {-13.125_in, 10.375_in},  //BackRightModule
                 frc::Translation2d {7.625_in, 10.375_in}, //FrontLeftModule
                 frc::Translation2d {7.625_in, -10.375_in}  //FrontRightModule
                                               }};
};
