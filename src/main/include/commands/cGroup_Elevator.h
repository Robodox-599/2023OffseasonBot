// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/subsystem_Elevator.h"
#include "subsystems/subsystem_Wrist.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_LEDs.h"

#include "commands/command_MoveElevator.h"
#include "commands/command_MoveWrist.h"
#include "commands/command_Intake.h"
#include "commands/command_Outtake.h"
#include "commands/command_FlashLEDs.h"

namespace ElevatorMovements{

  // Teleop Commands
  frc2::CommandPtr ScoreHighCone(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreMidCone(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreHighCube(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreMidCube(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreHybrid(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ToStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);

  frc2::CommandPtr GroundIntake(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr DoubleSubstationIntake(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);

  // Auton Commands
  frc2::CommandPtr ScoreHighConeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreMidConeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreHighCubeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);
  frc2::CommandPtr ScoreMidCubeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED);

}