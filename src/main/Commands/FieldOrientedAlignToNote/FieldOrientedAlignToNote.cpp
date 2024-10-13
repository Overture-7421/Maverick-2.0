
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FieldOrientedAlignToNote.h"
#include "Subsystems/Intake/Constants.h"
#include "Subsystems/Storage/Constants.h"

FieldOrientedAlignToNote::FieldOrientedAlignToNote(Chassis *chassis, photon::PhotonCamera *noteTrackingCamera, Intake *intake, Storage *storage, SuperStructure *superStructure) : noteTracking(chassis, noteTrackingCamera) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->chassis = chassis;
  this->camera = noteTrackingCamera;
  this->intake = intake;
  this->storage = storage;
  this->superStructure = superStructure;
  
  if(!frc::DriverStation::IsAutonomous){
    AddRequirements({ intake, storage, superStructure});
  } else {
    AddRequirements({});
  }
  
}

// Called when the command is initially scheduled.
void FieldOrientedAlignToNote::Initialize() {
  chassis->enableSpeedHelper(&noteTracking);
  if(!frc::DriverStation::IsAutonomous){
    superStructure->getTargetPosition(-31_deg, 68_deg);
    intake->setVoltage(ConstantsIn::NoteTrackingIn);
    storage->setVoltage(ConstantsSt::NoteTrackingSt);
  }
  
}

// Called repeatedly when this Command is scheduled to run   
void FieldOrientedAlignToNote::Execute() {
  if(!frc::DriverStation::IsAutonomous){
    if(storage->isNoteOnSensor()){
      storage->setVoltage(ConstantsSt::stopVoltage);
      intake->setVoltage(ConstantsIn::stopVolts);
    }
  }

}

// Called once the command ends or is interrupted.
void FieldOrientedAlignToNote::End(bool interrupted) {
  chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool FieldOrientedAlignToNote::IsFinished() {
  if(storage->isNoteOnSensor()){
    return true;
  }
  return false;
}

