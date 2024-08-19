#include "Chassis.h"

// Inicialización de miembros estáticos
frc::SimpleMotorFeedforward<units::meters> Chassis::feedForwardFrontLeft{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> Chassis::feedForwardFrontRight{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> Chassis::feedForwardBackLeft{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
frc::SimpleMotorFeedforward<units::meters> Chassis::feedForwardBackRight{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};

Chassis::Chassis() 
    : SwerveChassis(),
      frontLeftModule(FrontLeftConfig()),
      frontRightModule(FrontRightConfig()),
      backLeftModule(BackLeftConfig()),
      backRightModule(BackRightConfig()) {
    frc::SmartDashboard::PutData("Chassis/Odometry", &field2d);
}

ModuleConfig Chassis::FrontLeftConfig() {
    ModuleConfig config{feedForwardFrontLeft};
    config.DrivedId = 1;
    config.TurnId = 2;
    config.CanCoderId = 3;
    config.ModuleName = "Front Left";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::FrontRightConfig() {
    ModuleConfig config{feedForwardFrontRight};
    config.DrivedId = 4;
    config.TurnId = 5;
    config.CanCoderId = 6;
    config.ModuleName = "Front Right";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::BackLeftConfig() {
    ModuleConfig config{feedForwardBackLeft};
    config.DrivedId = 7;
    config.TurnId = 8;
    config.CanCoderId = 9;
    config.ModuleName = "Back Left";
    config.Offset = 0_deg;
    return config;
}

ModuleConfig Chassis::BackRightConfig() {
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
