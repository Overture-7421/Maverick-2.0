#include "ManualSpeakerCommand.h" // Asegúrate de incluir la clase SuperStructure
#include "Subsystems/Shooter/Constants.h"

ManualSpeakerCommand::ManualSpeakerCommand(SuperStructure* superstructure, Shooter* shooter){
  this->superstructure = superstructure;
  this->shooter = shooter;
  // Declara dependencias del subsistema.
  AddRequirements({superstructure, shooter});
}

// Inicializa el comando, moviendo las partes superior e inferior a las posiciones deseadas.
void ManualSpeakerCommand::Initialize() {
  superstructure->setToAngle(-15_deg, 60_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterSpeaker);
}

// Ejecutar cada ciclo del comando (no es necesario en este caso).
void ManualSpeakerCommand::Execute() {}

// Finaliza el comando si es interrumpido o terminado.
void ManualSpeakerCommand::End(bool interrupted) {}

// Retorna true cuando el comando debe finalizar.
bool ManualSpeakerCommand::IsFinished() {
  if(superstructure->getTargetPosition(-15_deg, 60_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterSpeaker)){
    return true;
  } else {
    return false;
  }

  // Podrías terminar el comando si alcanzaste la posición deseada.
  // O podrías usar una condición específica para terminar el comando.
}
