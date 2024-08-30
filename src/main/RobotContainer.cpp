#include "RobotContainer.h"

RobotContainer::RobotContainer() {
    // Set up the default command for the chassis
    //chassis.SetDefaultCommand(swerveDriveCommand);

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Bindings can be added here if needed, e.g., to trigger other commands.
}
