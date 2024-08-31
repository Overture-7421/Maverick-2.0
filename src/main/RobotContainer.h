#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "OvertureLib/Gamepad/Gamepad.h"
#include <frc/XboxController.h>
#include "Subsystems/Chassis/Chassis.h"
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h"

class RobotContainer {
public:
    RobotContainer();

private:
    // Controllers
    frc::XboxController xbox1{0};
    Gamepad driver{0,0.0,0.0};

    // Subsystems
    //Chassis chassis;


    // Helpers
    
    //HeadingSpeedsHelper headingHelper{headingController, &chassis};

    // Commands
    //SwerveDriveCommand swerveDriveCommand{&chassis, &driver, &headingHelper};

    void ConfigureButtonBindings();
};
