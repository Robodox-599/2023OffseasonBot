// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.subsystem_Elevator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class command_MoveElevator extends CommandBase {
  /** Creates a new command_MoveElevator. */
  private subsystem_Elevator m_Elevator;
  private DoubleSupplier m_EncPosition;
  private BooleanSupplier m_IsWait;
  private DoubleSupplier m_Threshold;

  private Timer m_Timer = new Timer();


  public command_MoveElevator(
    subsystem_Elevator elevator,
    DoubleSupplier encPosition,
    BooleanSupplier isWait,
    DoubleSupplier threshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = elevator;
    m_EncPosition = encPosition;
    m_IsWait = isWait;
    m_Threshold = threshold;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Initialize", true);
    m_Timer.start();
    m_Elevator.setElevatorPosition(m_EncPosition.getAsDouble());
    SmartDashboard.putNumber("Enc Pos", m_EncPosition.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_Elevator.isElevatorAtDesiredPosition()){
      m_Timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_IsWait.getAsBoolean() && m_Timer.get() < Constants.ElevatorConstants.maxTimeAllowed){
      return m_Elevator.elevatorThreshold(m_Threshold.getAsDouble());
    }
    else {
      return true;
    }
  }
}
