
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FieldOrientedAlignToNote.h"

FieldOrientedAlignToNote::FieldOrientedAlignToNote(Chassis *chassis, photon::PhotonCamera *noteTrackingCamera, Intake *intake, Storage *storage, SuperStructure *superStructure) : noteTracking(chassis, noteTrackingCamera) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->chassis = chassis;
  this->camera = noteTrackingCamera;
  this->intake = intake;
  this->storage = storage;
  this->superStructure = superStructure;
  
  AddRequirements({chassis, intake, storage, superStructure});
}

// Called when the command is initially scheduled.
void FieldOrientedAlignToNote::Initialize() {
  chassis->enableSpeedHelper(&noteTracking);
  superStructure->getTargetPosition(-31_deg, 68_deg);
  intake->startIntake();
  storage->startStorage();
}

// Called repeatedly when this Command is scheduled to run   
void FieldOrientedAlignToNote::Execute() {
  if(storage->isNoteOnSensor()){
    storage->stopStorage();
    intake->stopIntake();
  }
}

// Called once the command ends or is interrupted.
void FieldOrientedAlignToNote::End(bool interrupted) {
  chassis->disableSpeedHelper();
}

// Returns true when the command should end.
bool FieldOrientedAlignToNote::IsFinished() {
  return false;
}
