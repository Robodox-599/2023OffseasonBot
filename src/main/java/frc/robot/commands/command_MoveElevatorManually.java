// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.subsystem_Elevator;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class command_MoveElevatorManually extends CommandBase {
  /** Creates a new command_MoveElevatorManually. */
  private subsystem_Elevator m_Elevator;
  private DoubleSupplier m_TriggerInput;

  public command_MoveElevatorManually(subsystem_Elevator elevator, DoubleSupplier triggerInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putBoolean("Manual joystick entered exceute",true);
    SmartDashboard.putNumber("Trigger input val", m_TriggerInput.getAsDouble());

    if(Math.abs(m_TriggerInput.getAsDouble()) > Constants.ControllerConstants.deadband){
      m_Elevator.moveElevatorManually(m_TriggerInput.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
