// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;

public class cGroup_Elevator{
  public static Command scoreHighCone(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(
      new command_MoveWrist(wrist, () -> {return 0;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorScoreHighCone;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      new command_MoveWrist(wrist, () -> {return WristConstants.highConePos;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_Outtake(intake, led)
    );
  }

  public static Command scoreMidCone(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(
      new command_MoveWrist(wrist, () -> {return 0;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorScoreMidCone;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      new command_MoveWrist(wrist, () -> {return WristConstants.midConePos;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_Outtake(intake, led)
    );
  }

  public static Command scoreHighCube(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(
      // new command_MoveWrist(wrist, () -> {return 0;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      wrist.zeroWrist(),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorScoreHighCube;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      new command_MoveWrist(wrist, () -> {return WristConstants.highCubePos;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_Outtake(intake, led)
    );
  }

  public static Command scoreMidCube(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(
      wrist.zeroWrist(),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorScoreMidCube;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      new command_MoveWrist(wrist, () -> {return WristConstants.midCubePos;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_Outtake(intake, led)
    );
  }

  public static Command scoreHybrid(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(
      new command_MoveWrist(wrist, () -> {return 0;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorScoreHybrid;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      new command_MoveWrist(wrist, () -> {return WristConstants.groundIntake;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_Outtake(intake, led)
    );
  }

  public static Command toStow(subsystem_Elevator elevator, subsystem_Wrist wrist){
    return Commands.sequence(
      new command_MoveWrist(wrist, () -> {return 0;}, () -> {return true;}, () -> {return WristConstants.wristThreshold;}),
      new command_MoveElevator(elevator, () -> {return ElevatorConstants.kElevatorStow;}, () -> {return false;}, () -> {return ElevatorConstants.elevatorThreshold;}),
      wrist.zeroWrist()
      );
  }

  public static Command groundIntake(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(new command_MoveWrist(wrist, () -> 0, () -> true, () -> WristConstants.wristThreshold),
                            new command_MoveElevator(elevator, () -> ElevatorConstants.kElevatorScoreHybrid, () -> false, () -> ElevatorConstants.elevatorThreshold),
                            new command_MoveWrist(wrist, () -> WristConstants.groundIntake, () -> true, () -> WristConstants.wristThreshold),
                            new command_Intake(intake, led));
  }

  public static Command doubleSubstationIntake(subsystem_Elevator elevator, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(new command_MoveWrist(wrist, () -> 0, () -> true, () -> WristConstants.wristThreshold),
                            new command_MoveElevator(elevator, () -> ElevatorConstants.kElevatorIntakeDoubleSubstation, () -> false, () -> ElevatorConstants.elevatorThreshold),
                            new command_MoveWrist(wrist, () -> WristConstants.substationPos, () -> true, () -> WristConstants.wristThreshold),
                            new command_Intake(intake, led));
  }

  // Auton Commands
  public static Command scoreHighConeStow(subsystem_Elevator elevtor, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(scoreHighCone(elevtor, wrist, intake, led), toStow(elevtor, wrist));
  }
  public static Command scoreMidConeStow(subsystem_Elevator elevtor, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(scoreMidCone(elevtor, wrist, intake, led), toStow(elevtor, wrist));
  }
  public static Command scoreHighCubeStow(subsystem_Elevator elevtor, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(scoreHighCube(elevtor, wrist, intake, led), toStow(elevtor, wrist));
  }
  public static Command scoreMidCubeStow(subsystem_Elevator elevtor, subsystem_Wrist wrist, subsystem_Intake intake, subsystem_LEDs led){
    return Commands.sequence(scoreMidCube(elevtor, wrist, intake, led), toStow(elevtor, wrist));
  }
}
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html