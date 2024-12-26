#include "UtilityFunctions.h"

bool isRedAlliance() {
	auto alliance = frc::DriverStation::GetAlliance();
	if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
		return true;
	}
	return false;
}

units::length::meter_t getDistanceToChassis(Chassis* chassis, frc::Pose2d targetPose) {
	return chassis->getEstimatedPose().Translation().Distance(targetPose.Translation());
}