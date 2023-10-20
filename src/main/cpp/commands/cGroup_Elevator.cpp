// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/cGroup_Elevator.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc2/command/WaitCommand.h>
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Teleop Commands
  frc2::CommandPtr ElevatorMovements::ScoreHighCone(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreHighCone;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::HighConePos;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Outtake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::ScoreMidCone(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreMidCone;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::MidConePos;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Outtake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::ScoreHighCube(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreHighCube;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::HighCubePos;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Outtake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::ScoreMidCube(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreMidCube;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::MidCubePos;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Outtake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::ScoreHybrid(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreHybrid;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::GroundIntake;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Outtake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::ToStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorStow;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                Wrist->ZeroWrist());
  }

  frc2::CommandPtr ElevatorMovements::GroundIntake(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorScoreHybrid;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::GroundIntake;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Intake(Intake, LED).ToPtr());
  }
  frc2::CommandPtr ElevatorMovements::DoubleSubstationIntake(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(command_MoveWrist(Wrist, [=]{return 0;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_MoveElevator(Elevator, [=]{return ElevatorConstants::kElevatorIntakeDoubleSubstation;}, [=]{return false;}, [=]{return ElevatorConstants::ElevatorThreshold;}).ToPtr(),
                                command_MoveWrist(Wrist, [=]{return WristConstants::SubstationPos;}, [=]{return true;}, [=]{return WristConstants::WristThreshold;}).ToPtr(),
                                command_Intake(Intake, LED).ToPtr());
  }

  // Auton Commands
  frc2::CommandPtr ElevatorMovements::ScoreHighConeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(ElevatorMovements::ScoreHighCone(Elevator, Wrist, Intake, LED), ElevatorMovements::ToStow(Elevator, Wrist, Intake, LED));
  }
  frc2::CommandPtr ElevatorMovements::ScoreMidConeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(ElevatorMovements::ScoreMidCone(Elevator, Wrist, Intake, LED), ElevatorMovements::ToStow(Elevator, Wrist, Intake, LED));
  }
  frc2::CommandPtr ElevatorMovements::ScoreHighCubeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(ElevatorMovements::ScoreHighCube(Elevator, Wrist, Intake, LED), ElevatorMovements::ToStow(Elevator, Wrist, Intake, LED));
  }
  frc2::CommandPtr ElevatorMovements::ScoreMidCubeStow(subsystem_Elevator* Elevator, subsystem_Wrist* Wrist, subsystem_Intake* Intake, subsystem_LED* LED){
    return frc2::cmd::Sequence(ElevatorMovements::ScoreMidCube(Elevator, Wrist, Intake, LED), ElevatorMovements::ToStow(Elevator, Wrist, Intake, LED));
  }