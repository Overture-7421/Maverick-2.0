#include "Chassis.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include <units/angular_acceleration.h>

// Initialize static members

frc::SimpleMotorFeedforward<units::meters> feedForwardFrontLeft{ 0.079568_V, 2.1328_V / 1_mps, 0.18437_V / 1_mps_sq };
frc::SimpleMotorFeedforward<units::meters> feedForwardFrontRight{ 0.059944_V, 2.2414_V / 1_mps, 0.16932_V / 1_mps_sq };
frc::SimpleMotorFeedforward<units::meters> feedForwardBackLeft{ 0.010764_V, 2.0759_V / 1_mps, 0.45385_V / 1_mps_sq };
frc::SimpleMotorFeedforward<units::meters> feedForwardBackRight{ 0.20934_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq };

Chassis::Chassis()
	: SwerveChassis() {
	configureSwerveBase();
	setAcceptingVisionMeasurements(true);
	resetHeading();
}

units::meters_per_second_t Chassis::getMaxModuleSpeed() {
	return 5_mps;
}

units::meter_t Chassis::getDriveBaseRadius() {
	return 0.5270_m;
}

SwerveModule& Chassis::getBackLeftModule() {
	return backLeftModule;
}

SwerveModule& Chassis::getBackRightModule() {
	return backRightModule;
}

SwerveModule& Chassis::getFrontLeftModule() {
	return frontLeftModule;
}

SwerveModule& Chassis::getFrontRightModule() {
	return frontRightModule;
}

frc::SlewRateLimiter<units::meters_per_second>& Chassis::getVxLimiter() {
	return vxLimiter;
}

frc::SlewRateLimiter<units::meters_per_second>& Chassis::getVyLimiter() {
	return vyLimiter;
}

frc::SlewRateLimiter<units::radians_per_second>& Chassis::getVwLimiter() {
	return vwLimiter;
}

frc::SwerveDriveKinematics<4>& Chassis::getKinematics() {
	return kinematics;
}

frc::Rotation2d Chassis::getRotation2d() {
	return pigeon.GetRotation2d();
}


frc::Rotation3d Chassis::getRotation3d() {
	return pigeon.GetRotation3d();
}

SwerveModuleConfig Chassis::FrontLeftConfig() {
	SwerveModuleConfig config{ feedForwardFrontLeft };
	config.DriveMotorConfig.MotorId = 6;
	config.TurnMotorConfig.MotorId = 5;
	config.EncoderConfig.CanCoderId = 11;
	config.CanBus = "OverCANivore";
	config.DriveGearRatio = 5.9027777;
	config.TurnGearRatio = 150.0 / 7.0;
	config.WheelDiameter = 4_in;
	config.DriveMotorConfig.PIDConfigs.kP = 53;
	config.ModuleName = "Front Left";
	config.EncoderConfig.Offset = -0.443359375_tr;
	config.DriveMotorConfig.Inverted = false;
	return config;
}

SwerveModuleConfig Chassis::FrontRightConfig() {
	SwerveModuleConfig config{ feedForwardFrontRight };
	config.DriveMotorConfig.MotorId = 8;
	config.TurnMotorConfig.MotorId = 7;
	config.EncoderConfig.CanCoderId = 12;
	config.CanBus = "OverCANivore";
	config.DriveGearRatio = 5.9027777;
	config.TurnGearRatio = 150.0 / 7.0;
	config.WheelDiameter = 4_in;
	config.DriveMotorConfig.PIDConfigs.kP = 53.0;
	config.DriveMotorConfig.Inverted = false;
	config.ModuleName = "Front Right";
	config.EncoderConfig.Offset = -0.13037109375_tr;
	return config;
}

SwerveModuleConfig Chassis::BackLeftConfig() {
	SwerveModuleConfig config{ feedForwardBackLeft };
	config.DriveMotorConfig.MotorId = 4;
	config.TurnMotorConfig.MotorId = 3;
	config.EncoderConfig.CanCoderId = 10;
	config.CanBus = "OverCANivore";
	config.DriveGearRatio = 5.9027777;
	config.TurnGearRatio = 150.0 / 7.0;
	config.WheelDiameter = 4_in;
	config.DriveMotorConfig.PIDConfigs.kP = 53.0; //Original 53
	config.ModuleName = "Back Left";
	config.DriveMotorConfig.Inverted = false;
	config.EncoderConfig.Offset = 0.05419921875_tr;
	return config;
}

SwerveModuleConfig Chassis::BackRightConfig() {
	SwerveModuleConfig config{ feedForwardBackRight };
	config.DriveMotorConfig.MotorId = 2;
	config.TurnMotorConfig.MotorId = 1;
	config.EncoderConfig.CanCoderId = 9;
	config.CanBus = "OverCANivore";
	config.DriveGearRatio = 5.9027777;
	config.TurnGearRatio = 150.0 / 7.0;
	config.WheelDiameter = 4_in;
	config.DriveMotorConfig.PIDConfigs.kP = 53.0;
	config.DriveMotorConfig.Inverted = false;
	config.ModuleName = "Back Right";
	config.EncoderConfig.Offset = 0.0791015625_tr;
	return config;
}
