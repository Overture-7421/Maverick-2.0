#include "ManualSpeakerCommand.h" // Asegúrate de incluir la clase SuperStructure

ManualSpeakerCommand::ManualSpeakerCommand(SuperStructure* superstructure)
    : superstructure{superstructure} {
  // Declara dependencias del subsistema.
  AddRequirements({superstructure});
}

// Inicializa el comando, moviendo las partes superior e inferior a las posiciones deseadas.
void ManualSpeakerCommand::Initialize() {
  superstructure->setToAngle(-15_deg, 60_deg);
}

// Ejecutar cada ciclo del comando (no es necesario en este caso).
void ManualSpeakerCommand::Execute() {}

// Finaliza el comando si es interrumpido o terminado.
void ManualSpeakerCommand::End(bool interrupted) {}

// Retorna true cuando el comando debe finalizar.
bool ManualSpeakerCommand::IsFinished() {
  // Podrías terminar el comando si alcanzaste la posición deseada.
  // O podrías usar una condición específica para terminar el comando.
  return true; 
}
