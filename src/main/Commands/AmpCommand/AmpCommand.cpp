#include "AmpCommand.h"
// Asegúrate de incluir la clase SuperStructure

AmpCommand::AmpCommand(SuperStructure* superstructure){
    this->superstructure = superstructure;
  // Declara dependencias del subsistema.
  AddRequirements({superstructure});
}


// Inicializa el comando, moviendo las partes superior e inferior a las posiciones deseadas.
void AmpCommand::Initialize() {
  superstructure->setAngle(70_deg, 65_deg);

}

// Ejecutar cada ciclo del comando (no es necesario en este caso).
void AmpCommand::Execute() {}

// Finaliza el comando si es interrumpido o terminado.
void AmpCommand::End(bool interrupted) {}

// Retorna true cuando el comando debe finalizar.
bool AmpCommand::IsFinished() {
  if(superstructure->getTargetPosition(70_deg, 65_deg)){
    return true;
  } else {
    return false;
  }

  // Podrías terminar el comando si alcanzaste la posición deseada.
  // O podrías usar una condición específica para terminar el comando.

}
