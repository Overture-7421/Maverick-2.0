#include "AmpCommand.h"
#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "Subsystems/Shooter/Constants.h"

// Asegúrate de incluir la clase SuperStructure

AmpCommand::AmpCommand(SuperStructure* superstructure, Shooter* shooter){
    this->superstructure = superstructure;
    this->shooter = shooter;
  // Declara dependencias del subsistema.
  AddRequirements({superstructure, shooter});
}


// Inicializa el comando, moviendo las partes superior e inferior a las posiciones deseadas.
void AmpCommand::Initialize() {
  superstructure->setToAngle(65_deg, 65_deg);
  shooter->setObjectiveVelocity(ConstantsSh::ShooterAmp);

}

// Ejecutar cada ciclo del comando (no es necesario en este caso).
void AmpCommand::Execute() {
}

// Finaliza el comando si es interrumpido o terminado.
void AmpCommand::End(bool interrupted) {}

// Retorna true cuando el comando debe finalizar.
bool AmpCommand::IsFinished() {
  if(superstructure->getTargetPosition(65_deg, 65_deg) && shooter->getObjectiveVelocity(ConstantsSh::ShooterAmp)){
    return true;
  } else {
    return false;
  }


  // Podrías terminar el comando si alcanzaste la posición deseada.
  // O podrías usar una condición específica para terminar el comando.

}
