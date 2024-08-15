#include "SuperStructure.h"

SuperStructure::SuperStructure() {

//Basically says motor 2, to follow motor one (ID, Bool if is inverted or not)

  lowerMotor2.setFollow(10, false);

//Adjust the gear ratio?

  lowerMotor1.setSensorToMechanism(0.0);  
  upperMotor.setSensorToMechanism(0.0);   

//Apply the PID values.

  lowerMotor1.setPIDValues(0.1, 0.0, 0.0, 0.0, 0.0);
  upperMotor.setPIDValues(0.1, 0.0, 0.0, 0.0, 0.0); 

}

//Command that moves to the designated lower position.
frc2::CommandPtr SuperStructure::moveToLowerPosition(double lowerPosition, double upperPosition) {
  return this->RunOnce([this, lowerPosition, upperPosition] {

  });
}

//Command that moves to the designated middle position.
frc2::CommandPtr SuperStructure::moveToMiddlePosition(double lowerPosition, double upperPosition) {
  return this->RunOnce([this, lowerPosition, upperPosition] {
  });
}

//Command that moves to the designated upperposition.
frc2::CommandPtr SuperStructure::moveToUpperPosition(double lowerPosition, double upperPosition) {
  return this->RunOnce([this, lowerPosition, upperPosition] {
  });
}


//Literally nothing lol

void SuperStructure::Periodic() {
}
