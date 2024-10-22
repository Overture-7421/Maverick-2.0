// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Storage.h"
#include <frc2/command/Commands.h>
#include "Constants.h"

Storage::Storage() = default;

// This method will be called once per scheduler run


void Storage::setVoltage(units::volt_t voltage){
    storagemotor.SetVoltage(voltage);
}

bool Storage::isNoteOnSensor(){
    return !(sensor1.Get());
    //return !(sensor2.Get());
}

frc2::CommandPtr Storage::startStorage(){
    return this->RunOnce([this] {this->setVoltage(ConstantsSt::startVoltage);});
};

frc2::CommandPtr Storage::startStorageAuto(){
    return this->RunOnce([this] {this->setVoltage(ConstantsSt::startVoltageAuto);});
};

frc2::CommandPtr Storage::stopStorage(){
    return this->RunOnce([this] {this->setVoltage(ConstantsSt::stopVoltage);});
};

frc2::CommandPtr Storage::reverseStorage(){
    return this->RunOnce([this] {this->setVoltage(ConstantsSt::reverseVoltage);});
};

void Storage::Periodic() {
    frc::SmartDashboard::PutBoolean("is note on sensor?? ", isNoteOnSensor());
}
