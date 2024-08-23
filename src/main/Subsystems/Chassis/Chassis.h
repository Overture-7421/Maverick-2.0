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
   OverPigeon pigeon{13}; 

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
    frc::SlewRateLimiter<units::meters_per_second> vxLimiter;
    frc::SlewRateLimiter<units::meters_per_second> vyLimiter;
    frc::SlewRateLimiter<units::radians_per_second> vwLimiter;

//POSE LOGS
    wpi::log::StructLogEntry<frc::Pose2d> poseLog;
    wpi::log::StructLogEntry<frc::Pose2d> visionPoseLog;

    // Kinematics for chassis configuration
     frc::Field2d field2d;
    frc::SwerveDriveKinematics<4> kinematics {{
                 frc::Translation2d{-1_m, 1_m},
                 frc::Translation2d{1_m, 1_m},
                 frc::Translation2d{-1_m, -1_m},
                 frc::Translation2d{1_m, -1_m}
                                               }};
    void updateOdometry();
};
