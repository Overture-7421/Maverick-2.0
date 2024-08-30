/*#include "SwerveDriveCommand.h"

SwerveDriveCommand::SwerveDriveCommand(Chassis* chassis, Gamepad* gamepad, HeadingSpeedsHelper* headingHelper)
    : chassis(chassis), gamepad(gamepad), headingHelper(headingHelper) {
  AddRequirements(chassis);
}

void SwerveDriveCommand::Execute() {
  // Get joystick inputs
  double forward = -gamepad->GetLeftY();  // Inverted if necessary
  double strafe = gamepad->GetLeftX();
  double rotation = gamepad->GetRightX();

  // Create chassis speeds based on joystick inputs
  frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      units::meters_per_second_t(forward),
      units::meters_per_second_t(strafe),
      units::radians_per_second_t(rotation),
      chassis->getEstimatedPose().Rotation());

  // Use the HeadingSpeedsHelper to alter the speed for maintaining heading
  headingHelper->alterSpeed(chassisSpeeds);

  // Drive the chassis with the calculated speeds
  chassis->Drive(chassisSpeeds);
}

void SwerveDriveCommand::End(bool interrupted) {
  // Stop the chassis when the command ends
  chassis->Drive(frc::ChassisSpeeds());
}

bool SwerveDriveCommand::IsFinished() {
  // This command never ends, as it's intended to be run while the robot is active
  return false;
}
*/