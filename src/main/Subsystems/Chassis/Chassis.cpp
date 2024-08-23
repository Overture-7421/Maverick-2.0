#include "Chassis.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include <units/angular_acceleration.h>

// Initialize static members

frc::SimpleMotorFeedforward<units::meters> feedForwardFrontLeft{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardFrontRight{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackLeft{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> feedForwardBackRight{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};

Chassis::Chassis()
    : SwerveChassis(),
      frontLeftModule(FrontLeftConfig()),
      frontRightModule(FrontRightConfig()),
      backLeftModule(BackLeftConfig()),
      backRightModule(BackRightConfig()),
      vxLimiter(18_mps_sq), 
      vyLimiter(18_mps_sq), 
      vwLimiter(12.5664_rad_per_s_sq),  
      poseLog(frc::DataLogManager::GetLog(), "/chassis/pose"),
      visionPoseLog(frc::DataLogManager::GetLog(), "/chassis/visionPose")
{
    frc::DataLogManager::Start();
}




void Chassis::Drive(const frc::ChassisSpeeds& speeds){
    // Convert the desired chassis speeds to individual module states using kinematics
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);
    // Set the module states using the base class methodsetModuleStates(moduleStates);
}

units::meters_per_second_t Chassis::getMaxModuleSpeed() {
    return 10_mps;
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
    config.DrivedId = 1;
    config.TurnId = 2;
    config.CanCoderId = 3;
    config.ModuleName = "Front Left";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::FrontRightConfig(){
    ModuleConfig config{feedForwardFrontRight};
    config.DrivedId = 4;
    config.TurnId = 5;
    config.CanCoderId = 6;
    config.ModuleName = "Front Right";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::BackLeftConfig(){
    ModuleConfig config{feedForwardBackLeft};
    config.DrivedId = 7;
    config.TurnId = 8;
    config.CanCoderId = 9;
    config.ModuleName = "Back Left";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::BackRightConfig(){
    ModuleConfig config{feedForwardBackRight};
    config.DrivedId = 10;
    config.TurnId = 11;
    config.CanCoderId = 12;
    config.ModuleName = "Back Right";
    config.Offset = 0_deg;
    return config;
}



void Chassis::shuffleboardPeriodic(){
    frc::SmartDashboard::PutNumber("Odometry/LinearX", getCurrentSpeeds().vx.value());
    frc::SmartDashboard::PutNumber("Odometry/LinearY", getCurrentSpeeds().vy.value());
    frc::SmartDashboard::PutNumber("Odometry/Angular", getCurrentSpeeds().omega.value());

    frc::SmartDashboard::PutNumber("Odometry/AccelX", getCurrentAccels().ax.value());
    frc::SmartDashboard::PutNumber("Odometry/AccelY", getCurrentAccels().ay.value());
    frc::SmartDashboard::PutNumber("Odometry/AccelOmega", getCurrentAccels().omega.value());

    frc::SmartDashboard::PutNumber("Odometry/X", getEstimatedPose().X().value());
    frc::SmartDashboard::PutNumber("Odometry/Y", getEstimatedPose().Y().value());
    frc::SmartDashboard::PutNumber("Odometry/Rotation", getEstimatedPose().Rotation().Degrees().value());

    frontLeftModule.shuffleboardPeriodic();
    frontRightModule.shuffleboardPeriodic();
    backLeftModule.shuffleboardPeriodic();
    backRightModule.shuffleboardPeriodic();
}
