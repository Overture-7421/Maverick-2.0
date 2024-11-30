#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/XboxController.h>
#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"


#include "Subsystems/Chassis/Chassis.h"
#include "Subsystems/Storage/Storage.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Shooter/Constants.h"
#include "Subsystems/SupportArms/SupportArms.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/WaitForButton/WaitForButton.h"
#include "Commands/SuperStructureMoveByDistance/SuperStructureMoveByDistance.h"
#include "Commands/PathFind/Pathfind.h"


#include <vector>
#include <utility>

frc2::CommandPtr AutoClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, OverXboxController* gamepad);
frc2::CommandPtr ManualClimb(Chassis* chassis, SuperStructure* superStructure, SupportArms* supportArms, Storage* storage, Shooter* shooter, OverXboxController* gamepad);
