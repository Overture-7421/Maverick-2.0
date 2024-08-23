#include "SuperStructure.h"
#include "Constants.h"

/*
SuperStructure::SuperStructure() {

//Basically says motor 2, to follow motor one (ID, Bool if is inverted or not)

  lowerLeftMotor.setFollow(lowerRightMotor.GetDeviceID(), true);

//setSensorToMechanism calculates how many rotor rotations it needs for one sensor rotation.

  lowerRightMotor.setSensorToMechanism(Constants::LowerSensorToMechanism); 
  upperMotor.setSensorToMechanism(Constants::UpperSensorToMechanism);   

//setRotorToSensorRatio calculates how many rotor rotations it needs for onw mechanism rotation. GearRatio.
  lowerRightMotor.setRotorToSensorRatio(Constants::LowerGearRatio);
  upperMotor.setRotorToSensorRatio(Constants::UpperGearRatio);  

  lowerRightMotor.setFusedCANCoder(Constants::LowerCANCoderID);
  upperMotor.setFusedCANCoder(Constants::UpperCANCoderID);

  lowerRightMotor.setClosedLoopVoltageRamp(Constants::VoltageRamp);
  upperMotor.setClosedLoopVoltageRamp(Constants::VoltageRamp);

  //limita pico de corriente inicial

  lowerRightMotor.setStatorCurrentLimit(true, Constants::StatorCurrentLimit);
  upperMotor.setStatorCurrentLimit(true, Constants::StatorCurrentLimit);

  lowerRightMotor.setSupplyCurrentLimit(true, Constants::SupplyCurrentLimit, Constants::TriggerThresholdCurrent, Constants::TriggerThresholdTime);
  upperMotor.setSupplyCurrentLimit(true, Constants::SupplyCurrentLimit, Constants::TriggerThresholdCurrent, Constants::TriggerThresholdTime);

  lowerRightMotor.configureMotionMagic(Constants::CruiseVelocity, Constants::CruiseAcceleration, 0.0);
  upperMotor.configureMotionMagic(Constants::CruiseVelocity, Constants::CruiseAcceleration, 0.0);

  lowerRightMotor.setContinuousWrap();
  upperMotor.setContinuousWrap();


//Apply the PID values.

  lowerRightMotor.setPIDValues(0.1, 0.0, 0.0, 0.0, 0.0);
  upperMotor.setPIDValues(0.1, 0.0, 0.0, 0.0, 0.0); 

  
}

//Command that moves to the designated lower position.
frc2::CommandPtr SuperStructure::setAngle(units::degree_t lowerAngle, units::degree_t upperAngle) {
    double actualLowerAngle = lowerAngle.value() / 360;
    double actualUpperAngle = upperAngle.value() / 360;
    lowerRightMotor.setMotionMagicPosition(actualLowerAngle, 0.0, true);
    upperMotor.setMotionMagicPosition(actualUpperAngle, 0.0, true);

  };


//Literally nothing lol

void SuperStructure::Periodic() {
}

*/