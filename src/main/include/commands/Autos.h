// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);
}  // namespace autos

/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_Elevator.h"
#include "subsystems/subsystem_Wrist.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_LEDs.h"

#include "commands/command_DriveAuton.h"
#include "commands/command_MoveElevator.h"
#include "commands/command_MoveWrist.h"
#include "commands/command_Intake.h"
#include "commands/command_Outtake.h"
#include "commands/command_FlashLEDs.h"

namespace autos {

    frc2::CommandPtr Taxi(subsystem_DriveTrain* DriveTrain);

    frc2::CommandPtr ScoreConeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
    frc2::CommandPtr One_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
    frc2::CommandPtr Two_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
    frc2::CommandPtr Three_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);

}  // namespace autos

*/