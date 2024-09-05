#include "WaitForButton.h"

frc2::CommandPtr WaitForButton(Gamepad* gamepad, int buttonNumber) {
	return frc2::cmd::WaitUntil([=]() {return gamepad->GetRawButton(buttonNumber);});
}