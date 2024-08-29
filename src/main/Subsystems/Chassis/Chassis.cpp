#include "Chassis.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include <units/angular_acceleration.h>

// Initialize static members

frc::SimpleMotorFeedforward<units::meters> feedForwardFrontLeft{0.079568_V, 2.1328_V / 1_mps, 0.18437_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardFrontRight{0.059944_V, 2.2414_V / 1_mps, 0.16932_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackLeft{0.010764_V, 2.0759_V / 1_mps, 0.45385_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackRight{0.20934_V, 2.0879_V / 1_mps, 0.098433_V / 1_mps_sq};

Chassis::Chassis()
    : SwerveChassis()
{
    configureSwerveBase();
    setAcceptingVisionMeasurements(true);
    pigeon.SetYaw(0_deg);
    resetHeading();
}

void Chassis::Drive(const frc::ChassisSpeeds& speeds){
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);
}

units::meters_per_second_t Chassis::getMaxModuleSpeed() {
    return 5_mps;
}

units::meter_t Chassis::getDriveBaseRadius() {
   return 1_m; 
}

SwerveModule& Chassis::getBackLeftModule(){
    return backLeftModule;
}

SwerveModule& Chassis::getBackRightModule(){
    return backRightModule;
}

SwerveModule& Chassis::getFrontLeftModule(){
    return frontLeftModule;
}

SwerveModule& Chassis::getFrontRightModule(){
    return frontRightModule;
}

frc::SlewRateLimiter<units::meters_per_second>& Chassis::getVxLimiter(){
    return vxLimiter;
}

frc::SlewRateLimiter<units::meters_per_second>& Chassis::getVyLimiter(){
    return vyLimiter;
}

frc::SlewRateLimiter<units::radians_per_second>& Chassis::getVwLimiter(){
    return vwLimiter;
}

frc::SwerveDriveKinematics<4>& Chassis::getKinematics(){
    return kinematics;
}

 wpi::log::StructLogEntry<frc::Pose2d>&  Chassis::getPoseLog(){
    return poseLog;
 }

wpi::log::StructLogEntry<frc::Pose2d>& Chassis::getVisionPoseLog(){
    return visionPoseLog;
}

frc::Rotation2d Chassis::getRotation2d() {
    return pigeon.GetRotation2d();
}


frc::Rotation3d Chassis::getRotation3d() {    
    return pigeon.GetRotation3d();
}



ModuleConfig Chassis::FrontLeftConfig(){
    ModuleConfig config{feedForwardFrontLeft};
    config.DrivedId = 6;
    config.TurnId = 5;
    config.CanCoderId = 11;
    config.DriveStatorCurrentLimit = 110;
    config.DriveCurrentLimit = 60;
    config.DriveTriggerThresholdTime = 0.5;
    config.DriveRampRate = 0.25;
    config.TurnStatorCurrentLimit = 80;
    config.TurnCurrentLimit = 60;
    config.TurnTriggerThresholdTime = 0.2;
    config.CanBus = "OverCANivore";
    config.DriveNeutralMode = ControllerNeutralMode::Brake;
    config.TurnNeutralMode = ControllerNeutralMode::Coast;
    config.DriveGearRatio = 5.9027777;
    config.TurnGearRatio = 150.0 / 7.0;
    config.WheelDiameter = 4_in;
    config.kP = 56.0; //Original 53
    config.ModuleName = "Front Left";
    config.Offset = -0.443359375_tr;
    config.DriveInverted = true;
    return config;
}

ModuleConfig Chassis::FrontRightConfig(){
    ModuleConfig config{feedForwardFrontRight};
    config.DrivedId = 8;
    config.TurnId = 7;
    config.CanCoderId = 12;
    config.DriveStatorCurrentLimit = 110;
    config.DriveCurrentLimit = 60;
    config.DriveTriggerThresholdTime = 0.5;
    config.DriveRampRate = 0.25;
    config.TurnStatorCurrentLimit = 80;
    config.TurnCurrentLimit = 60;
    config.TurnTriggerThresholdTime = 0.2;
    config.CanBus = "OverCANivore";
    config.DriveNeutralMode = ControllerNeutralMode::Brake;
    config.TurnNeutralMode = ControllerNeutralMode::Coast;
    config.DriveGearRatio = 5.9027777;
    config.TurnGearRatio = 150.0 / 7.0;
    config.WheelDiameter = 4_in;
    config.kP = 53.0;
    config.DriveInverted = true;
    config.ModuleName = "Front Right";
    config.Offset = -0.13037109375_tr;
    return config;
}

ModuleConfig Chassis::BackLeftConfig(){
    ModuleConfig config{feedForwardBackLeft};
    config.DrivedId = 4;
    config.TurnId = 3;
    config.CanCoderId = 10;
    config.DriveStatorCurrentLimit = 110;
    config.DriveCurrentLimit = 60;
    config.DriveTriggerThresholdTime = 0.5;
    config.DriveRampRate = 0.25;
    config.TurnStatorCurrentLimit = 80;
    config.TurnCurrentLimit = 60;
    config.TurnTriggerThresholdTime = 0.2;
    config.CanBus = "OverCANivore";
    config.DriveNeutralMode = ControllerNeutralMode::Brake;
    config.TurnNeutralMode = ControllerNeutralMode::Coast;
    config.DriveGearRatio = 5.9027777;
    config.TurnGearRatio = 150.0 / 7.0;
    config.WheelDiameter = 4_in;
    config.kP = 53.0; //Original 53
    config.ModuleName = "Back Left";
    config.DriveInverted = true;
    config.Offset = 0.05419921875_tr;
    return config;
}

ModuleConfig Chassis::BackRightConfig(){
    ModuleConfig config{feedForwardBackRight};
    config.DrivedId = 2;
    config.TurnId = 1;
    config.CanCoderId = 9;
    config.DriveStatorCurrentLimit = 110;
    config.DriveCurrentLimit = 60;
    config.DriveTriggerThresholdTime = 0.5;
    config.DriveRampRate = 0.25;
    config.TurnStatorCurrentLimit = 80;
    config.TurnCurrentLimit = 60;
    config.TurnTriggerThresholdTime = 0.2;
    config.CanBus = "OverCANivore";
    config.DriveNeutralMode = ControllerNeutralMode::Brake;
    config.TurnNeutralMode = ControllerNeutralMode::Coast;
    config.DriveGearRatio = 5.9027777;
    config.TurnGearRatio = 150.0 / 7.0;
    config.WheelDiameter = 4_in;
    config.kP = 53.0;
    config.DriveInverted = true;
    config.ModuleName = "Back Right";
    config.Offset = 0.0791015625_tr;
    return config;
}

 void Chassis::shuffleboardPeriodic(){
    frc::SmartDashboard::PutNumber("Odometry/LinearX",
			desiredSpeeds.vx.value());
	frc::SmartDashboard::PutNumber("Odometry/LinearY",
			desiredSpeeds.vy.value());
	frc::SmartDashboard::PutNumber("Odometry/Angular",
			desiredSpeeds.omega.value());

	frc::SmartDashboard::PutNumber("Odometry/AccelX", currentAccels.ax.value());
	frc::SmartDashboard::PutNumber("Odometry/AccelY", currentAccels.ay.value());
	frc::SmartDashboard::PutNumber("Odometry/AccelOmega",
			currentAccels.omega.value());

	frc::SmartDashboard::PutNumber("Odometry/SpeedX", desiredSpeeds.vx.value());
	frc::SmartDashboard::PutNumber("Odometry/SpeedY", desiredSpeeds.vy.value());
	frc::SmartDashboard::PutNumber("Odometry/SpeedOmega",
			desiredSpeeds.omega.value());

	field2d.SetRobotPose(latestPose);
	frc::SmartDashboard::PutNumber("Odometry/X", latestPose.X().value());
	frc::SmartDashboard::PutNumber("Odometry/Y", latestPose.Y().value());
	frc::SmartDashboard::PutNumber("Odometry/Rotation",
			latestPose.Rotation().Degrees().value());

	getFrontLeftModule().shuffleboardPeriodic();
	getFrontRightModule().shuffleboardPeriodic();
	getBackRightModule().shuffleboardPeriodic();
	getBackLeftModule().shuffleboardPeriodic();
}
