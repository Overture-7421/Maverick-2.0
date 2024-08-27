#include "SuperStructure.h"
#include <iostream>



SuperStructure::SuperStructure() {
  auto getConfig = upperCANCoder.getConfiguration();
  getConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
  upperCANCoder.GetConfigurator().Apply(getConfig);

  auto getLowerConfig = lowerCANCoder.getConfiguration();
  getLowerConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
  lowerCANCoder.GetConfigurator().Apply(getLowerConfig);
//Basically says motor 2, to follow motor one (ID, Bool if is inverted or not)

   lowerLeftMotor.setFollow(lowerRightMotor.GetDeviceID(), true);

// setSensorToMechanism calculates how many sensor it needs for one mechanism rotation. 

   lowerRightMotor.setSensorToMechanism(ConstantsSS::LowerSensorToMechanism); 
   upperMotor.setSensorToMechanism(ConstantsSS::UpperSensorToMechanism);   

// //setRotorToSensorRatio calculates how many rotor rotations it needs for onw sensor rotation. GearRatio.
   lowerRightMotor.setRotorToSensorRatio(ConstantsSS::LowerGearRatio);
   upperMotor.setRotorToSensorRatio(ConstantsSS::UpperGearRatio);  

   lowerRightMotor.setFusedCANCoder(ConstantsSS::LowerCANCoderID);
   upperMotor.setFusedCANCoder(ConstantsSS::UpperCANCoderID);

   lowerRightMotor.setClosedLoopVoltageRamp(ConstantsSS::VoltageRamp);
   upperMotor.setClosedLoopVoltageRamp(ConstantsSS::VoltageRamp);

//   //limita pico de corriente inicial

   //lowerRightMotor.setStatorCurrentLimit(true, Constants::StatorCurrentLimit);
   //upperMotor.setStatorCurrentLimit(true, Constants::StatorCurrentLimit);

   //lowerRightMotor.setSupplyCurrentLimit(true, Constants::SupplyCurrentLimit, Constants::TriggerThresholdCurrent, Constants::TriggerThresholdTime);
   //upperMotor.setSupplyCurrentLimit(true, Constants::SupplyCurrentLimit, Constants::TriggerThresholdCurrent, Constants::TriggerThresholdTime);

   lowerRightMotor.setContinuousWrap();
   upperMotor.setContinuousWrap();


// //Apply the PID values.

   lowerRightMotor.setPIDValues(100, 10, 0.0, 0.0, 0.0);
   upperMotor.setPIDValues(100, 60, 0.0, 0.0, 0.0); 

   lowerRightMotor.configureMotionMagic(ConstantsSS::CruiseVelocity, ConstantsSS::CruiseAcceleration, 0.0);
   upperMotor.configureMotionMagic(0.5, 2, 0.0);

  
}

//Command that moves to the designated lower position.
void SuperStructure::setToAngle(units::degree_t lowerAngle, units::degree_t upperAngle) {
    double actualLowerAngle = lowerAngle.value() / 360.00;
    double actualUpperAngle = upperAngle.value() / 360.00;

    auto turns = units::turn_t(lowerRightMotor.GetClosedLoopReference().GetValueAsDouble());
    auto turnsPerSecond = units::turns_per_second_t(lowerRightMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    auto wristTurns = units::turn_t(upperMotor.GetClosedLoopReference().GetValueAsDouble());
    auto wristTurnsPerSecond = units::turns_per_second_t(upperMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    lowerRightMotor.setMotionMagicPosition(actualLowerAngle, armFeedForward.Calculate(turns, turnsPerSecond).value(), true);
    upperMotor.setMotionMagicPosition(actualUpperAngle, wristFeedForward.Calculate(wristTurns, wristTurnsPerSecond).value(), true);
  };

//feedForward.Calculate(turns, turnsPerSecond).value()

frc2::CommandPtr SuperStructure::setAngle(units::degree_t lowerAngle, units::degree_t upperAngle){
  return this->RunOnce([this, lowerAngle, upperAngle] {
    this->setToAngle(lowerAngle, upperAngle);
    });
  
  }

void SuperStructure::getCurrentAngle(double lowerAngle, double upperAngle){
    double currentLowerAngle = lowerAngle * 360;
    double currentUpperAngle = upperAngle * 360;
    frc::SmartDashboard::PutNumber("current lower angle", currentLowerAngle);
    frc::SmartDashboard::PutNumber("current upper angle", currentUpperAngle);
   };

frc2::CommandPtr SuperStructure::SysIdQuasistatic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr SuperStructure::SysIdDynamic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Dynamic(direction);
}


//Literally nothing lol

void SuperStructure::Periodic() {
}
