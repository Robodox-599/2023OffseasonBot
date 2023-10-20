// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

frc2::CommandPtr autos::Taxi(subsystem_DriveTrain* DriveTrain){
  return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, "Kasparov (4T)", true).ToPtr());
}

frc2::CommandPtr autos::ScoreConeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
  
}
frc2::CommandPtr autos::One_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){

}
frc2::CommandPtr autos::Two_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){

}
frc2::CommandPtr autos::Three_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){

}
*/
