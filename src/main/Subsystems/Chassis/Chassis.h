// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"


class Chassis: public SwerveChassis {
 public:
  Chassis();

	virtual units::meters_per_second_t getMaxModuleSpeed() = 0;
	virtual units::meter_t getDriveBaseRadius() = 0;

	virtual frc::Rotation2d getRotation2d() = 0;
	virtual frc::Rotation3d getRotation3d() = 0;
  
  	virtual SwerveModule& getFrontLeftModule() = 0;
	virtual SwerveModule& getFrontRightModule() = 0;
	virtual SwerveModule& getBackLeftModule() = 0;
	virtual SwerveModule& getBackRightModule() = 0;

	virtual frc::SlewRateLimiter<units::meters_per_second>& getVxLimiter() = 0;
	virtual frc::SlewRateLimiter<units::meters_per_second>& getVyLimiter() = 0;
	virtual frc::SlewRateLimiter<units::radians_per_second>& getVwLimiter() = 0;

	virtual frc::SwerveDriveKinematics<4>& getKinematics() = 0;

	virtual wpi::log::StructLogEntry<frc::Pose2d>& getPoseLog() = 0;
	virtual wpi::log::StructLogEntry<frc::Pose2d>& getVisionPoseLog() = 0;
	bool configuredChassis = false;

	void shuffleboardPeriodic();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
	frc::SwerveDriveKinematics<4> kinematics {{
		frc::Translation2d(1_mm, 1_m),
		frc::Translation2d(1_mm, 1_m),
		frc::Translation2d(1_mm, 1_m),
		frc::Translation2d(1_mm, 1_m)
	}};

	frc::SlewRateLimiter<units::meters_per_second> VxLimiter {1_mps_sq};
	frc::SlewRateLimiter<units::meters_per_second> VyLimiter {1_mps_sq};
	frc::SlewRateLimiter<units::radians_per_second> VwLimiter {1_rad_per_s_sq};


	frc::SimpleMotorFeedforward<units::meters> FeedForward { 0_V, 0_V / 1_mps,
			0_V / 1_mps_sq };
	ModuleConfig moduleConfig {FeedForward};

	SwerveModule frontLeftModule {moduleConfig};
	SwerveModule frontRighttModule {moduleConfig};
	SwerveModule backLeftModule {moduleConfig};
SwerveModule backLeftModule {moduleConfig};


	

	//OverPigeon overPigeon;
	//frc::Rotation2d overPigeon(13, "OverCANivore");
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
