#include "Cllimbing.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/PathFind/Pathfind.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include <exception>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "SpeedsHelpers/ClimbingSpeedHelper/ClimbingSpeedHelper.h"

int checkpointButtonId = frc::XboxController::Button::kBack;

units::degree_t lowerStartingState = -15_deg;
units::degree_t upperStartingState = 30_deg;
units::degree_t lowerTargetState = 85_deg;
units::degree_t upperTargetState = 10_deg;
SuperStructureMoveByDistance::Profile superStructureProfile{ lowerStartingState, upperStartingState, lowerTargetState, upperTargetState, 1.25_m };

units::second_t storageTrapScoreWait = 1_s;



frc2::CommandPtr GoToClimbingLocationPathFind(SuperStructure* superStructure, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow, Chassis* chassis) {
	return frc2::cmd::Deadline(
		Pathfind(chassis, pathToFollow->getStartingHolonomicPose().value()).ToPtr(),
		superStructure->setAngle(lowerStartingState, upperStartingState)
	);
}

frc2::CommandPtr GoToClimbingLocationOnTheFly(std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow) {
	return pathplanner::AutoBuilder::followPath(pathToFollow);
}

frc2::CommandPtr SetUpJoints(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, std::shared_ptr<pathplanner::PathPlannerPath> pathToFollow, OverXboxController* gamepad) {

	const auto& poses = pathToFollow->getPathPoses();
	auto targetPos = poses[poses.size() - 1];

	std::function<units::meter_t()> distanceFunction = [=]() {
		if (isRedAlliance() && !pathToFollow->preventFlipping) {
			return getDistanceToChassis(chassis, pathplanner::FlippingUtil::flipFieldPose(targetPos));
		} else {
			return getDistanceToChassis(chassis, targetPos);
		}
	};

	return frc2::cmd::Sequence(
		superStructure->setAngle(lowerStartingState, upperStartingState),
		WaitForButton(gamepad, checkpointButtonId),
		frc2::cmd::Deadline(
			pathplanner::AutoBuilder::followPath(pathToFollow),
			SuperStructureMoveByDistance(superStructure, superStructureProfile, distanceFunction).ToPtr(),
			supportArms->servoAngleCommand(10_deg)
		)
	);
}


frc2::CommandPtr ClimbAtLocation(SuperStructure* superStructure, Shooter* shooter, Storage* storage, OverXboxController* gamepad) {
	return frc2::cmd::Sequence(
		superStructure->setAngle(-31_deg, 85_deg),
		frc2::cmd::WaitUntil([=] {return superStructure->getTargetPosition(-31_deg, 85_deg);}).WithTimeout(1_s),
		WaitForButton(gamepad, checkpointButtonId),
		superStructure->setAngle(93_deg, 14_deg),
		frc2::cmd::WaitUntil([=] {return superStructure->getTargetPosition(93_deg, 14_deg);}).WithTimeout(1_s),
		frc2::cmd::RunOnce([=] { shooter->setVoltage(ConstantsSh::TrapVolts);}),
		WaitForButton(gamepad, checkpointButtonId),
		storage->startStorage(),
		frc2::cmd::Wait(1_s),
		WaitForButton(gamepad, checkpointButtonId),
		superStructure->setAngle(-10_deg, 80_deg),
		frc2::cmd::WaitUntil([=] {return superStructure->getTargetPosition(-10_deg, 80_deg);}).WithTimeout(1_s)
		// frc2::cmd::RunOnce([=] { shooter->setVoltage(0.0);})
	);
}

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, OverXboxController* gamepad) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathLeft = pathplanner::PathPlannerPath::fromPathFile("Climb Left");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathRight = pathplanner::PathPlannerPath::fromPathFile("Climb Right");
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathBack = pathplanner::PathPlannerPath::fromPathFile("Climb Back");

	return frc2::cmd::Select<StageLocation>([chassis]() {return findClosestStageLocation(chassis);},
		std::pair{ StageLocation::Left, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, climbPathLeft, chassis).AndThen([=] { chassis->setAcceptingVisionMeasurements(false);}),
			WaitForButton(gamepad, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathLeft, gamepad),
			WaitForButton(gamepad, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage, gamepad)
		) },
		std::pair{ StageLocation::Right, frc2::cmd::Sequence(
			GoToClimbingLocationPathFind(superStructure, climbPathRight, chassis).AndThen([=] { chassis->setAcceptingVisionMeasurements(false);}),
			WaitForButton(gamepad, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathRight, gamepad),
			WaitForButton(gamepad, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage, gamepad)
		) },
		std::pair{ StageLocation::Back, frc2::cmd::Sequence(
			frc2::cmd::RunOnce([=]() {
				chassis->setAcceptingVisionMeasurements(false);
			}),
			GoToClimbingLocationPathFind(superStructure, climbPathBack, chassis).AndThen([=] { chassis->setAcceptingVisionMeasurements(false);}),
			WaitForButton(gamepad, checkpointButtonId),
			SetUpJoints(chassis, superStructure, supportArms, climbPathBack, gamepad),
			WaitForButton(gamepad, checkpointButtonId),
			ClimbAtLocation(superStructure, shooter, storage, gamepad)
		) }
	).FinallyDo(
		[=]() {
		chassis->setAcceptingVisionMeasurements(true);
	}
	);
};

frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, OverXboxController* gamepad) {
	static std::shared_ptr<pathplanner::PathPlannerPath> climbPathManual = pathplanner::PathPlannerPath::fromPathFile("Climb Manual");
	climbPathManual->preventFlipping = true;

	return frc2::cmd::Sequence(
		frc2::cmd::RunOnce([=]() {
		chassis->setAcceptingVisionMeasurements(false);
		chassis->resetOdometry(climbPathManual->getStartingDifferentialPose());
	}),
		SetUpJoints(chassis, superStructure, supportArms, climbPathManual, gamepad),
		WaitForButton(gamepad, checkpointButtonId),
		ClimbAtLocation(superStructure, shooter, storage, gamepad)
	);
}